import numpy as np

class ScaraNewtonRaphson:
    def __init__(self, L1=45.0, L2=50.0):
        """
        Initialize the 2-link SCARA robot parameters.
        """
        self.L1 = L1
        self.L2 = L2
    def is_reachable(self, pt):
        """Checks if a 2D target point (x, y) falls within the robot's physical workspace bounds."""
        r = np.hypot(pt[0], pt[1])
        r_min = abs(self.L1 - self.L2)
        r_max = self.L1 + self.L2
        return r_min <= r <= r_max

    def forward_kinematics(self, theta):
        """
        Calculates the current end-effector coordinates f(theta).
        """
        t1, t2 = theta
        x = self.L1 * np.cos(t1) + self.L2 * np.cos(t1 + t2)
        y = self.L1 * np.sin(t1) + self.L2 * np.sin(t1 + t2)
        return np.array([x, y])

    def jacobian(self, theta):
        """
        Calculates the analytical Jacobian matrix J(theta).
        This maps joint velocities to end-effector velocities.
        """
        t1, t2 = theta
        
        # Partial derivatives of x and y with respect to t1 and t2
        dx_dt1 = -self.L1 * np.sin(t1) - self.L2 * np.sin(t1 + t2)
        dx_dt2 = -self.L2 * np.sin(t1 + t2)
        
        dy_dt1 = self.L1 * np.cos(t1) + self.L2 * np.cos(t1 + t2)
        dy_dt2 = self.L2 * np.cos(t1 + t2)
        
        return np.array([
            [dx_dt1, dx_dt2],
            [dy_dt1, dy_dt2]
        ])

    def inverse_kinematics(self, target_pos, theta_guess, tol=1e-4, max_iter=100):
        """
        Newton-Raphson numerical inverse kinematics algorithm.
        
        Parameters:
            target_pos : list or array of [x_d, y_d]
            theta_guess: initial guess for [theta_1, theta_2] in radians
            tol        : error tolerance in mm
            max_iter   : maximum number of iterations allowed
            
        Returns:
            theta_i    : the calculated joint angles
            success    : boolean indicating if it converged
        """
        theta_i = np.array(theta_guess, dtype=float)
        x_d = np.array(target_pos, dtype=float)

        for i in range(max_iter):
            # 1. Calculate f(theta_i): Current end-effector position
            current_pos = self.forward_kinematics(theta_i)
            
            # 2. Calculate error vector e = x_d - f(theta_i)
            error = x_d - current_pos
            
            # 3. Check if error is within the acceptable tolerance
            if np.linalg.norm(error) < tol:
                print(f"IK Converged in {i} iterations.")
                return theta_i, True
            
            # 4. Calculate Jacobian at current guess J(theta_i)
            J = self.jacobian(theta_i)
            
            # 5. Use the Moore-Penrose pseudoinverse to handle singularities
            # As mentioned in the transcript, this minimizes the error if 
            # the exact target is unreachable.
            J_pinv = np.linalg.pinv(J)
            
            # 6. Update guess: theta_{i+1} = theta_i + J_pinv * e
            delta_theta = J_pinv @ error
            theta_i = theta_i + delta_theta
            
        print("IK Failed to converge within maximum iterations.")
        return theta_i, False


if __name__ == "__main__":
    # --- Example Usage ---
    
    # Create the robot solver
    robot = ScaraNewtonRaphson(L1=45, L2=50)
    
    # Define a desired configuration x_d (e.g., x=41.57, y=67.22)
    desired_position = [41.57, 67.22]
    
    # Define an initial guess theta_zero (e.g., both joints at 0 degrees)
    initial_guess = [0.0, 0.0]
    
    print(f"Target Position: {desired_position}")
    print(f"Initial Guess (rad): {initial_guess}")
    print("-" * 30)
    
    # Run the Newton-Raphson Algorithm
    q_sol, success = robot.inverse_kinematics(desired_position, initial_guess)
    
    if success:
        print("-" * 30)
        print("Solution found (Radians):", np.round(q_sol, 4))
        print("Solution found (Degrees):", np.round(np.degrees(q_sol), 2))
        
        # Verify the solution by plugging it back into forward kinematics
        verification = robot.forward_kinematics(q_sol)
        print("Verification Position: ", np.round(verification, 2))