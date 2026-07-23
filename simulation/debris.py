import numpy as np


class Debris:

    def __init__(self, points, speeds, start_time=0.0,radius=0.1):

        self.start_time = float(start_time)
        self.radius = radius
    
        if len(points) == 0:
            raise ValueError("Debris requires at least one point.")

        if len(points) - 1 != len(speeds):
            raise ValueError("Debris requires exactly one velocity per segment.")

        self.points = [np.asarray(point, dtype=float) for point in points]

        self.speeds = speeds

        self.velocity_vectors = []

        for i in range(len(self.points)-1):
            direction = (self.points[i+1] - self.points[i])

            direction /= np.linalg.norm(direction)

            self.velocity_vectors.append(self.speeds[i]*direction)


        self.segment_start_times = []
        self.segment_end_times = []

        current_time = self.start_time

        for i in range(len(self.points) - 1):

            distance = np.linalg.norm(self.points[i + 1] - self.points[i])
            duration = distance / self.speeds[i]

            self.segment_start_times.append(current_time)
            current_time += duration
            self.segment_end_times.append(current_time)

        self.end_time = current_time

    def is_active(self, time):

        return (self.start_time <= time <= self.end_time)

    def get_position(self, time):

        if time <= self.start_time:
            return self.points[0]

        if time >= self.end_time:
            return self.points[-1]

        for i in range(len(self.speeds)):

            if self.segment_start_times[i] <= time <= self.segment_end_times[i]:

                return (
                    self.points[i]
                    + self.velocity_vectors[i]
                    * (time - self.segment_start_times[i])
                )

        raise RuntimeError(
            "Unable to determine debris position."
        )

    def get_velocity(self, time):

        if not self.is_active(time):
            return np.zeros(2)

        for i in range(len(self.speeds)):

            if self.segment_start_times[i] <= time <= self.segment_end_times[i]:

                return self.velocity_vectors[i]

        return np.zeros(2)