import numpy as np  

A_final = np.array([[0.95085546, 0.29124849, 0.10511047, 0.301],
                    [0.19840159, -0.83371212, 0.51532603, -0.139],
                    [0.2377198, -0.46914648, -0.85052388, 0.266]]).T.reshape(-1, 1)  # Add your desired pose here
print(A_final)