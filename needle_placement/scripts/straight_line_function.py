import numpy as np
   
def interpolate(A_entry, A_ball, t):
        """
        Interpolate between the two poses
        Args:
            t: The interpolation parameter, ranging from 0 (at A_entry) to 1 (at A_ball)
        Returns:
            Pose matrix of the interpolated pose
        """
        assert 0.0 <= t <= 1.0, "Interpolation parameter should be between 0 and 1"
        return (1.0 - t) * A_entry + t * A_ball

if __name__ == "__main__":
    A_entry = np.array([[ 0.95663454,  0.28769805,  0.04560904, 0.283],
                            [ 0.23035644, -0.84302106,  0.486057, -0.208],
                            [ 0.17828703, -0.45447258, -0.87273616, 0.366],
                            [ 0.0, 0.0, 0.0, 1.0]])

    A_ball = np.array([[0.95085546, 0.29124849, 0.10511047, 0.301],
                       [0.19840159, -0.83371212, 0.51532603, -0.139],
                       [0.2377198, -0.46914648, -0.85052388, 0.266],
                       [0.0, 0.0, 0.0, 1.0]])


    for t in np.linspace(0, 1, 10):
        interpolated_pose = interpolate(A_entry, A_ball, t)
        print(interpolated_pose)
