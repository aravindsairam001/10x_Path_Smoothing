"""
Test Trajectory Control Algorithms
Unit tests for trajectory control functionality.
"""

import unittest
import numpy as np
import sys
import os

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from trajectory_control import DifferentialDrive, PurePursuitController, StanleyController, PIDController
from trajectory_control.differential_drive import RobotParameters


class TestDifferentialDrive(unittest.TestCase):
    """Test cases for differential drive robot model."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.robot = DifferentialDrive()
        self.robot.set_state(0, 0, 0)
    
    def test_initialization(self):
        """Test robot initialization."""
        robot = DifferentialDrive()
        state = robot.get_state()
        
        self.assertEqual(state.x, 0.0)
        self.assertEqual(state.y, 0.0)
        self.assertEqual(state.theta, 0.0)
        self.assertEqual(state.v, 0.0)
        self.assertEqual(state.omega, 0.0)
    
    def test_state_setting(self):
        """Test setting robot state."""
        self.robot.set_state(1, 2, np.pi/4, 1.5, 0.5)
        state = self.robot.get_state()
        
        self.assertEqual(state.x, 1.0)
        self.assertEqual(state.y, 2.0)
        self.assertEqual(state.theta, np.pi/4)
        self.assertEqual(state.v, 1.5)
        self.assertEqual(state.omega, 0.5)
    
    def test_kinematics_update(self):
        """Test kinematic model update."""
        # Move forward
        self.robot.update_kinematics(1.0, 0.0, dt=1.0)
        pose = self.robot.get_pose()
        
        self.assertAlmostEqual(pose[0], 1.0, places=5)  # x should be 1
        self.assertAlmostEqual(pose[1], 0.0, places=5)  # y should be 0
        self.assertAlmostEqual(pose[2], 0.0, places=5)  # theta should be 0
    
    def test_wheel_velocity_conversion(self):
        """Test wheel velocity conversions."""
        # Test body to wheel velocity conversion
        v_left, v_right = self.robot.body_velocities_to_wheel_velocities(1.0, 0.5)
        
        # For positive angular velocity, right wheel should be faster
        self.assertGreater(v_right, v_left)
        
        # Test wheel to body velocity conversion
        v_body, omega_body = self.robot.wheel_velocities_to_body_velocities(v_left, v_right)
        
        self.assertAlmostEqual(v_body, 1.0, places=5)
        self.assertAlmostEqual(omega_body, 0.5, places=5)
    
    def test_velocity_limits(self):
        """Test velocity limiting."""
        # Test limiting excessive velocities
        v_limited, omega_limited = self.robot.limit_velocities(10.0, 10.0)
        
        self.assertLessEqual(abs(v_limited), self.robot.params.max_linear_velocity)
        self.assertLessEqual(abs(omega_limited), self.robot.params.max_angular_velocity)


class TestPurePursuitController(unittest.TestCase):
    """Test cases for Pure Pursuit controller."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.robot = DifferentialDrive()
        self.robot.set_state(0, 0, 0)
        self.controller = PurePursuitController(lookahead_distance=2.0)
        
        # Simple straight path
        self.straight_path = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]
        
        # Curved path
        self.curved_path = [(0, 0), (1, 0), (2, 1), (3, 1), (4, 0)]
    
    def test_lookahead_point_finding(self):
        """Test lookahead point calculation."""
        robot_pose = (0, 0, 0)
        
        lookahead_x, lookahead_y, _ = self.controller.find_lookahead_point(
            robot_pose, self.straight_path, current_velocity=1.0)
        
        # Should find a point ahead on the path
        self.assertGreater(lookahead_x, 0)
        self.assertEqual(lookahead_y, 0)  # On straight path
    
    def test_curvature_calculation(self):
        """Test curvature calculation."""
        robot_pose = (0, 0, 0)
        lookahead_point = (2, 0)
        
        curvature = self.controller.calculate_curvature(robot_pose, lookahead_point)
        
        # For straight ahead, curvature should be zero
        self.assertAlmostEqual(curvature, 0.0, places=3)
    
    def test_control_output(self):
        """Test control command generation."""
        v_cmd, omega_cmd = self.controller.control(self.robot, self.straight_path, 1.0)
        
        # Should produce valid control commands
        self.assertIsInstance(v_cmd, (int, float))
        self.assertIsInstance(omega_cmd, (int, float))
        self.assertGreaterEqual(v_cmd, 0)  # Should move forward
    
    def test_goal_reaching(self):
        """Test goal reaching detection."""
        robot_pose = (4, 0, 0)
        goal = (4, 0)
        
        reached = self.controller.is_goal_reached(robot_pose, goal, tolerance=0.5)
        self.assertTrue(reached)
        
        # Test not reached
        robot_pose = (0, 0, 0)
        reached = self.controller.is_goal_reached(robot_pose, goal, tolerance=0.5)
        self.assertFalse(reached)


class TestStanleyController(unittest.TestCase):
    """Test cases for Stanley controller."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.robot = DifferentialDrive()
        self.robot.set_state(0, 0, 0)
        self.controller = StanleyController(k_e=0.5)
        self.straight_path = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]
    
    def test_heading_error_calculation(self):
        """Test heading error calculation."""
        robot_heading = 0.0
        path_heading = np.pi/4
        
        heading_error = self.controller.calculate_heading_error(robot_heading, path_heading)
        
        self.assertAlmostEqual(heading_error, np.pi/4, places=5)
    
    def test_control_output(self):
        """Test control command generation."""
        v_cmd, omega_cmd = self.controller.control(self.robot, self.straight_path, 1.0)
        
        # Should produce valid control commands
        self.assertIsInstance(v_cmd, (int, float))
        self.assertIsInstance(omega_cmd, (int, float))


class TestPIDController(unittest.TestCase):
    """Test cases for PID controller."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.robot = DifferentialDrive()
        self.robot.set_state(0, 0, 0)
        self.controller = PIDController(kp_lateral=1.0, kp_heading=2.0)
        self.straight_path = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]
    
    def test_pid_control_calculation(self):
        """Test PID control calculation."""
        error = 1.0
        error_integral = 0.0
        error_previous = 0.5
        
        output, new_integral = self.controller.pid_control(
            error, error_integral, error_previous, 1.0, 0.1, 0.05, 0.1)
        
        # Should produce reasonable output
        self.assertIsInstance(output, (int, float))
        self.assertIsInstance(new_integral, (int, float))
    
    def test_control_output(self):
        """Test control command generation."""
        v_cmd, omega_cmd = self.controller.control(self.robot, self.straight_path, 1.0)
        
        # Should produce valid control commands
        self.assertIsInstance(v_cmd, (int, float))
        self.assertIsInstance(omega_cmd, (int, float))
    
    def test_parameter_tuning(self):
        """Test parameter tuning."""
        original_kp = self.controller.kp_lateral
        
        self.controller.tune_parameters(kp_lateral=2.0)
        
        self.assertEqual(self.controller.kp_lateral, 2.0)
        self.assertNotEqual(self.controller.kp_lateral, original_kp)
    
    def test_controller_reset(self):
        """Test controller reset."""
        # Set some non-zero internal state
        self.controller.lateral_error_integral = 1.0
        self.controller.path_index = 5
        
        self.controller.reset()
        
        self.assertEqual(self.controller.lateral_error_integral, 0.0)
        self.assertEqual(self.controller.path_index, 0)


if __name__ == '__main__':
    unittest.main()
