#!/usr/bin/env python3
"""
High-level manipulation functions for Franka FR3
Simple functions for common tasks: pick, place, move, etc.
"""

from .robot import FrankaHelperReal, create_pose
import time


class FrankaManipulation:
    """Simple manipulation task functions"""
    
    def __init__(self, helper: FrankaHelperReal):
        """
        Args:
            helper: FrankaHelperReal instance
        """
        self.helper = helper
        self.logger = helper.logger
    
    # Movement functions
    
    def move_to_position(self, x: float, y: float, z: float,
                        rx: float = 0, ry: float = 3.14159, rz: float = 0,
                        velocity_scaling: float = 0.5) -> bool:
        """
        Move end-effector to specific position (Cartesian)
        
        Args:
            x, y, z: Position in meters (relative to base)
            rx, ry, rz: Rotation in radians (Euler angles)
            velocity_scaling: Speed 0.0-1.0 (0.1=slow, 0.5=medium, 1.0=fast)
        
        Returns:
            True if successful
        
        Example:
            # Move slowly to position above table (30cm above, centered)
            move_to_position(0.3, 0.0, 0.5, velocity_scaling=0.2)
            
            # Move fast forward and up
            move_to_position(0.4, 0.2, 0.6, velocity_scaling=0.8)
        """
        self.logger.info(f"Moving to position: x={x:.3f}, y={y:.3f}, z={z:.3f} at {velocity_scaling*100:.0f}% speed")
        
        pose = create_pose(x=x, y=y, z=z, rx=rx, ry=ry, rz=rz)
        return self.helper.move_to_pose(pose, velocity_scaling=velocity_scaling)
    
    def move_to_joints(self, angles: list, velocity_scaling: float = 0.5) -> bool:
        """
        Move to joint configuration
        
        Args:
            angles: List of 7 joint angles in radians
            velocity_scaling: Speed 0.0-1.0 (0.1=slow, 0.5=medium, 1.0=fast)
        
        Returns:
            True if successful
        
        Example:
            # Move slowly to home
            move_to_joints([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], velocity_scaling=0.2)
            
            # Move ready (bent forward) at medium speed
            move_to_joints([0.0, 0.0, 0.0, -1.571, 0.0, 1.571, 0.785])
        """
        self.logger.info(f"Moving to joint configuration at {velocity_scaling*100:.0f}% speed")
        return self.helper.move_to_joints(angles, velocity_scaling=velocity_scaling)
    
    def move_home(self) -> bool:
        """Move to home position (safe, neutral)"""
        self.logger.info("Moving to HOME")
        return self.helper.move_to_named_state("home")
    
    def move_ready(self) -> bool:
        """Move to ready position (bent forward, ready to work)"""
        self.logger.info("Moving to READY")
        return self.helper.move_to_named_state("ready")
    
    # ============================================================================
    # GRIPPER FUNCTIONS
    # ============================================================================
    
    def open_gripper(self, width: float = 0.08) -> bool:
        """
        Open gripper
        
        Args:
            width: Opening width in meters (0.0=closed, 0.08=fully open)
        
        Returns:
            True if successful
        
        Example:
            open_gripper()           # Open fully (80mm)
            open_gripper(0.05)       # Open to 50mm
            open_gripper(0.04)       # Open to 40mm
        """
        self.logger.info(f"Opening gripper to {width*1000:.0f}mm")
        return self.helper.open_gripper(width=width)
    
    def close_gripper(self) -> bool:
        """
        Close gripper (full closure)
        
        Returns:
            True if successful
        """
        self.logger.info("Closing gripper")
        return self.helper.close_gripper()
    
    # Gripper functions
    
    def pick(self, x: float, y: float, z_approach: float, z_grasp: float,
             width: float = 0.04, wait_after: float = 1.0) -> bool:
        """
        Pick an object at given position
        
        Args:
            x, y: Position in meters (top-down view)
            z_approach: Height to approach from (meters)
            z_grasp: Height to grasp at (meters)
            width: Gripper opening before grasping (meters)
            wait_after: Wait time after closing gripper (seconds)
        
        Returns:
            True if successful
        
        Example - Pick object on table:
            # Object at (0.3m, 0.2m), table height 0.15m
            pick(x=0.3, y=0.2, z_approach=0.3, z_grasp=0.15)
            
            # Small object needing more precision
            pick(x=0.3, y=0.2, z_approach=0.25, z_grasp=0.12, width=0.03)
        """
        self.logger.info(f"PICKING object at ({x:.2f}, {y:.2f})")
        
        # Step 1: Open gripper
        self.logger.info("  [1/5] Opening gripper")
        self.open_gripper(width=width)
        time.sleep(1)
        
        # Step 2: Move to approach height
        self.logger.info(f"  [2/5] Moving to approach height")
        self.move_to_position(x, y, z_approach)
        time.sleep(1)
        
        # Step 3: Move down to grasp height
        self.logger.info(f"  [3/5] Moving to grasp height")
        self.move_to_position(x, y, z_grasp)
        time.sleep(0.5)
        
        # Step 4: Close gripper
        self.logger.info(f"  [4/5] Closing gripper")
        self.close_gripper()
        time.sleep(wait_after)
        
        # Step 5: Lift object
        self.logger.info(f"  [5/5] Lifting object")
        self.move_to_position(x, y, z_approach)
        time.sleep(1)
        
        self.logger.info("Pick complete")
        return True
    
    def place(self, x: float, y: float, z_approach: float, z_place: float,
              wait_after: float = 0.5) -> bool:
        """
        Place object at given position
        
        Args:
            x, y: Position in meters
            z_approach: Height to approach from (meters)
            z_place: Height to place at (meters)
            wait_after: Wait after opening gripper (seconds)
        
        Returns:
            True if successful
        
        Example - Place on table:
            # Place on table at (0.4m, 0.2m), table height 0.15m
            place(x=0.4, y=0.2, z_approach=0.3, z_place=0.15)
        """
        self.logger.info(f"PLACING object at ({x:.2f}, {y:.2f})")
        
        # Step 1: Move to approach height
        self.logger.info("  [1/4] Moving to approach height")
        self.move_to_position(x, y, z_approach)
        time.sleep(1)
        
        # Step 2: Move down to place height
        self.logger.info(f"  [2/4] Moving to place height")
        self.move_to_position(x, y, z_place)
        time.sleep(0.5)
        
        # Step 3: Open gripper
        self.logger.info(f"  [3/4] Opening gripper")
        self.open_gripper(width=0.08)
        time.sleep(wait_after)
        
        # Step 4: Lift away
        self.logger.info(f"  [4/4] Lifting away")
        self.move_to_position(x, y, z_approach)
        time.sleep(1)
        
        self.logger.info("Place complete")
        return True
    
    def pick_and_place(self, pick_x: float, pick_y: float,
                      place_x: float, place_y: float,
                      z_approach: float = 0.3, z_grasp: float = 0.15) -> bool:
        """
        Pick object from one location and place at another
        
        Args:
            pick_x, pick_y: Pick position
            place_x, place_y: Place position
            z_approach: Approach height
            z_grasp: Grasp height
        
        Returns:
            True if successful
        
        Example - Move object:
            pick_and_place(
                pick_x=0.3, pick_y=0.1,
                place_x=0.3, place_y=0.3
            )
        """
        self.logger.info("PICK AND PLACE operation")
        
        self.pick(pick_x, pick_y, z_approach, z_grasp)
        self.place(place_x, place_y, z_approach, z_grasp)
        
        self.logger.info("✓ Pick and place complete")
        return True
    
    # Helper methods
    
    def demo_sequence(self) -> bool:
        """Run a complete demo sequence"""
        self.logger.info("\n" + "="*70)
        self.logger.info("RUNNING DEMO SEQUENCE")
        self.logger.info("="*70 + "\n")
        
        try:
            # Home
            self.logger.info("[DEMO 1] Move to home")
            self.move_home()
            time.sleep(2)
            
            # Ready
            self.logger.info("\n[DEMO 2] Move to ready")
            self.move_ready()
            time.sleep(2)
            
            # Move to position
            self.logger.info("\n[DEMO 3] Move to position")
            self.move_to_position(0.3, 0.2, 0.4)
            time.sleep(2)
            
            # Gripper test
            self.logger.info("\n[DEMO 4] Gripper operations")
            self.open_gripper(0.08)
            time.sleep(1)
            self.open_gripper(0.05)
            time.sleep(1)
            self.close_gripper()
            time.sleep(1)
            
            # Pick simulation (no actual object)
            self.logger.info("\n[DEMO 5] Pick simulation")
            self.pick(x=0.3, y=0.2, z_approach=0.4, z_grasp=0.2)
            time.sleep(2)
            
            # Place simulation
            self.logger.info("\n[DEMO 6] Place simulation")
            self.place(x=0.4, y=0.3, z_approach=0.4, z_place=0.2)
            time.sleep(2)
            
            # Return home
            self.logger.info("\n[DEMO 7] Return to home")
            self.move_home()
            time.sleep(2)
            
            self.logger.info("\n" + "="*70)
            self.logger.info("DEMO SEQUENCE COMPLETE")
            self.logger.info("="*70 + "\n")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Demo failed: {e}")
            return False


def main(args=None):
    """Test manipulation functions"""
    import rclpy
    from rclpy.node import Node
    
    rclpy.init(args=args)
    node = Node("manipulation_test")
    
    # Create helper and manipulation interface
    helper = FrankaHelperReal(node)
    manip = FrankaManipulation(helper)
    
    # Run demo
    manip.demo_sequence()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
