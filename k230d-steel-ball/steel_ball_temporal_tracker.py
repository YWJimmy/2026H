"""Short-term tracking and bounded latency compensation for the steel ball.

Observed coordinates remain unsmoothed.  Velocity is used to bridge one
missing frame and, on request, to move an output box towards the estimated
current position.  Compensation has a stationary dead zone, a gradual
low-speed ramp, reversal protection, and a hard one-frame limit.
"""


ACQUIRE_CONFIDENCE = 0.13
TRACK_CONFIDENCE = 0.11
MAX_PREDICTION_FRAMES = 1
MAX_COMPENSATION_FRAMES = 1.0
VELOCITY_FILTER_ALPHA = 0.35
COMPENSATION_ENABLE_SPEED_PX_PER_FRAME = 5.0
COMPENSATION_DISABLE_SPEED_PX_PER_FRAME = 2.0
FULL_COMPENSATION_SPEED_PX_PER_FRAME = 12.0
REVERSAL_GUARD_SPEED_PX_PER_FRAME = 3.0
MAX_CENTER_DISTANCE = 220
OUTPUT_WIDTH = 1280
OUTPUT_HEIGHT = 960
MIN_BOX_SIZE = 20
MAX_BOX_SIZE = 160
MIN_ASPECT_RATIO = 0.35
MAX_ASPECT_RATIO = 2.80
PIPE_CENTER_Y_MIN = 100
PIPE_CENTER_Y_MAX = 860


def _center(box):
    return ((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0)


def _valid_geometry(detection):
    if detection is None or len(detection) != 5:
        return False
    x1, y1, x2, y2, _ = detection
    width = x2 - x1
    height = y2 - y1
    if width < MIN_BOX_SIZE or height < MIN_BOX_SIZE:
        return False
    if width > MAX_BOX_SIZE or height > MAX_BOX_SIZE:
        return False
    aspect_ratio = width / float(height)
    if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
        return False
    center_y = (y1 + y2) / 2.0
    return PIPE_CENTER_Y_MIN <= center_y <= PIPE_CENTER_Y_MAX


def _shift_box(box, delta_x, delta_y, score=None):
    width = box[2] - box[0]
    height = box[3] - box[1]
    x1 = int(round(box[0] + delta_x))
    y1 = int(round(box[1] + delta_y))
    x1 = max(0, min(OUTPUT_WIDTH - 1 - width, x1))
    y1 = max(0, min(OUTPUT_HEIGHT - 1 - height, y1))
    shifted_score = box[4] if score is None else score
    return [x1, y1, x1 + width, y1 + height, shifted_score]


class ShortTermTracker:
    """Accept quantized low-score detections and bridge one missing frame."""

    def __init__(self):
        self.last_box = None
        self.last_observed_center = None
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.missed_frames = 0
        self.last_source = "lost"
        self.compensation_allowed = True
        self.compensation_active = False

    def reset(self):
        self.last_box = None
        self.last_observed_center = None
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.missed_frames = 0
        self.last_source = "lost"
        self.compensation_allowed = True
        self.compensation_active = False

    def _expected_distance_squared(self, detection):
        center_x, center_y = _center(detection)
        prediction_steps = self.missed_frames + 1
        expected_x = self.last_observed_center[0] + self.velocity_x * prediction_steps
        expected_y = self.last_observed_center[1] + self.velocity_y * prediction_steps
        delta_x = center_x - expected_x
        delta_y = center_y - expected_y
        return delta_x * delta_x + delta_y * delta_y

    def _motion_is_plausible(self, detection):
        return self._expected_distance_squared(detection) <= MAX_CENTER_DISTANCE ** 2

    def _update_compensation_hysteresis(self):
        speed_squared = (
            self.velocity_x * self.velocity_x
            + self.velocity_y * self.velocity_y
        )
        if self.compensation_active:
            disable_squared = COMPENSATION_DISABLE_SPEED_PX_PER_FRAME ** 2
            if speed_squared <= disable_squared:
                self.compensation_active = False
        else:
            enable_squared = COMPENSATION_ENABLE_SPEED_PX_PER_FRAME ** 2
            if speed_squared >= enable_squared:
                self.compensation_active = True

    def _accept_observation(self, detection):
        center_x, center_y = _center(detection)
        if self.last_observed_center is None:
            self.velocity_x = 0.0
            self.velocity_y = 0.0
            self.compensation_allowed = True
            self.compensation_active = False
        else:
            previous_velocity_x = self.velocity_x
            previous_velocity_y = self.velocity_y
            elapsed_frames = self.missed_frames + 1
            new_velocity_x = (
                center_x - self.last_observed_center[0]
            ) / elapsed_frames
            new_velocity_y = (
                center_y - self.last_observed_center[1]
            ) / elapsed_frames
            previous_speed_squared = (
                previous_velocity_x * previous_velocity_x
                + previous_velocity_y * previous_velocity_y
            )
            new_speed_squared = (
                new_velocity_x * new_velocity_x
                + new_velocity_y * new_velocity_y
            )
            reversal_speed_squared = REVERSAL_GUARD_SPEED_PX_PER_FRAME ** 2
            direction_dot_product = (
                previous_velocity_x * new_velocity_x
                + previous_velocity_y * new_velocity_y
            )
            reversed_direction = (
                previous_speed_squared >= reversal_speed_squared
                and new_speed_squared >= reversal_speed_squared
                and direction_dot_product < 0.0
            )
            if reversed_direction:
                self.velocity_x = 0.0
                self.velocity_y = 0.0
                self.compensation_allowed = False
                self.compensation_active = False
            else:
                self.compensation_allowed = True
                if previous_speed_squared == 0.0:
                    self.velocity_x = new_velocity_x
                    self.velocity_y = new_velocity_y
                else:
                    alpha = VELOCITY_FILTER_ALPHA
                    self.velocity_x = (
                        alpha * new_velocity_x
                        + (1.0 - alpha) * previous_velocity_x
                    )
                    self.velocity_y = (
                        alpha * new_velocity_y
                        + (1.0 - alpha) * previous_velocity_y
                    )
                self._update_compensation_hysteresis()
        self.last_observed_center = (center_x, center_y)
        self.last_box = list(detection)
        self.missed_frames = 0
        self.last_source = "observed"
        return self.last_box

    def _handle_missing(self):
        if self.last_box is None or self.missed_frames >= MAX_PREDICTION_FRAMES:
            self.reset()
            return None
        self.missed_frames += 1
        predicted = _shift_box(
            self.last_box,
            self.velocity_x * self.missed_frames,
            self.velocity_y * self.missed_frames,
            score=0.0,
        )
        self.last_source = "predicted"
        return predicted

    def compensate(self, detection, latency_ms, frame_interval_ms):
        """Return a latency-compensated copy without changing tracker state."""
        if detection is None:
            return None
        if (
            not self.compensation_allowed
            or not self.compensation_active
            or frame_interval_ms <= 0.0
        ):
            return list(detection)

        speed_squared = (
            self.velocity_x * self.velocity_x
            + self.velocity_y * self.velocity_y
        )
        speed = speed_squared ** 0.5
        speed_range = (
            FULL_COMPENSATION_SPEED_PX_PER_FRAME
            - COMPENSATION_DISABLE_SPEED_PX_PER_FRAME
        )
        speed_scale = (
            speed - COMPENSATION_DISABLE_SPEED_PX_PER_FRAME
        ) / speed_range
        speed_scale = max(0.0, min(1.0, speed_scale))
        compensation_frames = latency_ms / frame_interval_ms
        compensation_frames = max(
            0.0,
            min(MAX_COMPENSATION_FRAMES, compensation_frames),
        )
        return _shift_box(
            detection,
            self.velocity_x * compensation_frames * speed_scale,
            self.velocity_y * compensation_frames * speed_scale,
        )

    def update_candidates(self, candidates):
        """Select exactly one valid single-target candidate and update state."""
        valid_candidates = [
            candidate for candidate in candidates if _valid_geometry(candidate)
        ]
        if self.last_box is None:
            acquisition_candidates = [
                candidate
                for candidate in valid_candidates
                if float(candidate[4]) >= ACQUIRE_CONFIDENCE
            ]
            if acquisition_candidates:
                selected = max(
                    acquisition_candidates,
                    key=lambda candidate: float(candidate[4]),
                )
                return self._accept_observation(selected)
        else:
            tracking_candidates = [
                candidate
                for candidate in valid_candidates
                if float(candidate[4]) >= TRACK_CONFIDENCE
                and self._motion_is_plausible(candidate)
            ]
            if tracking_candidates:
                selected = min(
                    tracking_candidates,
                    key=lambda candidate: (
                        self._expected_distance_squared(candidate),
                        -float(candidate[4]),
                    ),
                )
                return self._accept_observation(selected)
        return self._handle_missing()

    def update(self, detection):
        candidates = [] if detection is None else [detection]
        return self.update_candidates(candidates)
