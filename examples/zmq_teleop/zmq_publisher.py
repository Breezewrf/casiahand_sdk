#!/usr/bin/env python3
"""
ZMQ Publisher for Hand Teleoperator
Sends joint position commands to the hand teleoperator via ZMQ.

Usage:
    python3 zmq_publisher.py --port-left 5555 --port-right 5556
    python3 zmq_publisher.py --endpoint-left tcp://*:5555 --endpoint-right tcp://*:5556
    python3 zmq_publisher.py --demo keyboard
"""

import argparse
import json
import math
import time

import zmq

CASIA_REAL_LEFT_PORT = 5555
CASIA_REAL_RIGHT_PORT = 5556
DEFAULT_LEFT_ENDPOINT = f"tcp://*:{CASIA_REAL_LEFT_PORT}"
DEFAULT_RIGHT_ENDPOINT = f"tcp://*:{CASIA_REAL_RIGHT_PORT}"


class HandCommandPublisher:
    """Publishes hand joint commands via ZMQ to separate ports."""

    def __init__(self, zmq_endpoint_left=DEFAULT_LEFT_ENDPOINT, zmq_endpoint_right=DEFAULT_RIGHT_ENDPOINT):
        """
        Initialize the ZMQ publisher with separate sockets for each hand.

        Args:
            zmq_endpoint_left: ZMQ endpoint for left hand (e.g., tcp://*:5555)
            zmq_endpoint_right: ZMQ endpoint for right hand (e.g., tcp://*:5556)
        """
        self.context = zmq.Context()
        
        # Create separate sockets for left and right hands
        self.socket_left = self.context.socket(zmq.PUB)
        self.socket_right = self.context.socket(zmq.PUB)
        
        self.socket_left.bind(zmq_endpoint_left)
        self.socket_right.bind(zmq_endpoint_right)
        
        self.endpoint_left = zmq_endpoint_left
        self.endpoint_right = zmq_endpoint_right

        # Joint configuration for CASIA Hand-M
        # 10 DOF per hand (5 fingers × 2 joints)
        self.num_dofs_per_hand = 10
        self.num_dofs = 20  # Total for both hands
        self.joint_names = self._create_joint_names()
        self.left_joint_names = self.joint_names[:10]
        self.right_joint_names = self.joint_names[10:20]

        print(f"[Publisher] Left hand bound to {zmq_endpoint_left}")
        print(f"[Publisher] Right hand bound to {zmq_endpoint_right}")
        print(f"[Publisher] Publishing for {self.num_dofs_per_hand} DOF per hand")
        
        # Allow subscribers to connect
        time.sleep(1)

    def _create_joint_names(self):
        """Create names in the CASIA motor order used by the C++ SDK."""
        suffixes = [
            "thumb_proximal",
            "thumb_intermediate",
            "index_proximal",
            "middle_proximal",
            "ring_proximal",
            "pinky_proximal",
            "index_intermediate",
            "middle_intermediate",
            "ring_intermediate",
            "pinky_intermediate",
        ]
        return [f"{side}_{suffix}" for side in ("left", "right") for suffix in suffixes]

    def publish_command_left(self, qpos, joint_names=None):
        """
        Publish a left hand joint command.

        Args:
            qpos: List of joint positions for left hand in radians
            joint_names: Optional list of joint names
        """
        if joint_names is None:
            joint_names = self.left_joint_names

        # Ensure qpos has correct length
        if len(qpos) != len(joint_names):
            print(
                f"[WARNING] left qpos length ({len(qpos)}) != "
                f"joint_names length ({len(joint_names)})"
            )
            qpos = qpos[: len(joint_names)]
            if len(qpos) < len(joint_names):
                qpos.extend([0.0] * (len(joint_names) - len(qpos)))

        # Create JSON message
        message = {
            "timestamp": time.time(),
            "qpos": list(qpos),
            "joint_names": joint_names,
        }

        # Serialize and send
        json_str = json.dumps(message)
        self.socket_left.send_string(json_str)

        print(f"[PUB-L] Sent: {json_str[:80]}..." if len(json_str) > 80 else f"[PUB-L] Sent: {json_str}")

    def publish_command_right(self, qpos, joint_names=None):
        """
        Publish a right hand joint command.

        Args:
            qpos: List of joint positions for right hand in radians
            joint_names: Optional list of joint names
        """
        if joint_names is None:
            joint_names = self.right_joint_names

        # Ensure qpos has correct length
        if len(qpos) != len(joint_names):
            print(
                f"[WARNING] right qpos length ({len(qpos)}) != "
                f"joint_names length ({len(joint_names)})"
            )
            qpos = qpos[: len(joint_names)]
            if len(qpos) < len(joint_names):
                qpos.extend([0.0] * (len(joint_names) - len(qpos)))

        # Create JSON message
        message = {
            "timestamp": time.time(),
            "qpos": list(qpos),
            "joint_names": joint_names,
        }

        # Serialize and send
        json_str = json.dumps(message)
        self.socket_right.send_string(json_str)

        print(f"[PUB-R] Sent: {json_str[:80]}..." if len(json_str) > 80 else f"[PUB-R] Sent: {json_str}")

    def publish_command(self, qpos_left, qpos_right, joint_names_left=None, joint_names_right=None):
        """
        Publish both left and right hand commands simultaneously.

        Args:
            qpos_left: List of joint positions for left hand
            qpos_right: List of joint positions for right hand
            joint_names_left: Optional joint names for left hand
            joint_names_right: Optional joint names for right hand
        """
        self.publish_command_left(qpos_left, joint_names_left)
        self.publish_command_right(qpos_right, joint_names_right)

    def publish_sine_wave(self, duration=10, frequency=0.5, amplitude=0.5):
        """
        Publish a sine wave motion for demonstration.

        Args:
            duration: Duration in seconds
            frequency: Frequency in Hz
            amplitude: Amplitude in radians
        """
        print(f"\n[Demo] Publishing sine wave motion for {duration} seconds")
        print(f"       Frequency: {frequency} Hz, Amplitude: {amplitude} rad\n")

        start_time = time.time()

        while time.time() - start_time < duration:
            t = time.time() - start_time

            # Create sine wave for each hand
            qpos_left = [
                amplitude * math.sin(2 * math.pi * frequency * t) 
                for _ in range(self.num_dofs_per_hand)
            ]
            qpos_right = [
                amplitude * math.sin(2 * math.pi * frequency * t + math.pi) 
                for _ in range(self.num_dofs_per_hand)
            ]

            self.publish_command(qpos_left, qpos_right)
            time.sleep(0.05)  # 20 Hz publishing rate

    def publish_grasp_motion(self, motion_time=2.0, repeat=3):
        """
        Publish a simple grasp/release motion for both hands.

        Args:
            motion_time: Time for each motion phase in seconds
            repeat: Number of repetitions
        """
        print(f"\n[Demo] Publishing grasp motion ({repeat} repetitions)\n")

        for rep in range(repeat):
            print(f"[Rep {rep + 1}/{repeat}] Closing hands...")

            # Closing phase - all fingers curl
            start_time = time.time()
            while time.time() - start_time < motion_time:
                progress = (time.time() - start_time) / motion_time
                # Smooth closing motion using sine
                qpos_left = [
                    1.57 * (0.5 - 0.5 * math.cos(math.pi * progress)) 
                    for _ in range(self.num_dofs_per_hand)
                ]
                qpos_right = [
                    1.57 * (0.5 - 0.5 * math.cos(math.pi * progress)) 
                    for _ in range(self.num_dofs_per_hand)
                ]
                self.publish_command(qpos_left, qpos_right)
                time.sleep(0.05)

            print(f"[Rep {rep + 1}/{repeat}] Opening hands...")

            # Opening phase - all fingers extend
            start_time = time.time()
            while time.time() - start_time < motion_time:
                progress = (time.time() - start_time) / motion_time
                # Smooth opening motion using sine
                qpos_left = [
                    1.57 * (0.5 + 0.5 * math.cos(math.pi * progress)) 
                    for _ in range(self.num_dofs_per_hand)
                ]
                qpos_right = [
                    1.57 * (0.5 + 0.5 * math.cos(math.pi * progress)) 
                    for _ in range(self.num_dofs_per_hand)
                ]
                self.publish_command(qpos_left, qpos_right)
                time.sleep(0.05)

            time.sleep(0.5)  # Pause between repetitions

    def publish_individual_finger_test(self):
        """Test each finger individually for both hands."""
        print("\n[Demo] Testing individual fingers for both hands\n")

        # Test each joint one at a time
        for joint_idx in range(self.num_dofs_per_hand):
            print(f"[Test] Activating left joint {joint_idx + 1} ({self.left_joint_names[joint_idx]})...")
            # Activate this joint on left hand
            qpos_left = [0.0] * self.num_dofs_per_hand
            qpos_left[joint_idx] = 0.785  # ~45 degrees
            
            if joint_idx == 0:
                print("       (Thumb 1st joint)")
                qpos_left[joint_idx] = 0.885
            
            qpos_right = [0.0] * self.num_dofs_per_hand
            
            for _ in range(10):  # Hold for ~0.5 seconds
                self.publish_command(qpos_left, qpos_right)
                time.sleep(0.05)

            # Reset
            qpos_left = [0.0] * self.num_dofs_per_hand
            self.publish_command(qpos_left, qpos_right)
            time.sleep(0.2)

    def publish_zero_positions(self, duration=5):
        """
        Publish zero positions (all joints at rest).

        Args:
            duration: Duration to publish in seconds
        """
        print(f"\n[Demo] Publishing zero positions for {duration} seconds\n")

        start_time = time.time()
        qpos_left = [0.0] * self.num_dofs_per_hand
        qpos_right = [0.0] * self.num_dofs_per_hand

        while time.time() - start_time < duration:
            self.publish_command(qpos_left, qpos_right)
            time.sleep(0.1)

    def run_keyboard_control(self, step_size=0.1):
        """
        Interactive keyboard control for joint positions.
        
        Commands:
            L1-L10: Select left hand joint
            R1-R10: Select right hand joint
            +/=:  Increase selected joint by step_size
            -:    Decrease selected joint by step_size
            c:    Show current state
            r:    Reset all joints to zero
            h:    Show help
            q:    Quit
        """
        qpos_left = [0.0] * self.num_dofs_per_hand
        qpos_right = [0.0] * self.num_dofs_per_hand
        selected_hand = 'left'  # 'left' or 'right'
        selected_joint = 0
        max_angle = math.pi
        
        print("\n" + "="*70)
        print("[Keyboard Control] Interactive Dual Hand Joint Control Mode")
        print("="*70)
        print("\nQuick Start:")
        print(f"  Type 'L1' to select Left Hand Joint 1 ({self.left_joint_names[0]})")
        print(f"  Type 'R1' to select Right Hand Joint 1 ({self.right_joint_names[0]})")
        print("  Type '+' to increase, '-' to decrease")
        print("  Type 'r' to reset, 'h' for help, 'q' to quit\n")
        
        try:
            while True:
                if selected_hand == 'left':
                    current_qpos = qpos_left[selected_joint]
                    joint_name = self.left_joint_names[selected_joint]
                    print(f"\n[Selected] Left Hand Joint {selected_joint+1}: {joint_name}")
                else:
                    current_qpos = qpos_right[selected_joint]
                    joint_name = self.right_joint_names[selected_joint]
                    print(f"\n[Selected] Right Hand Joint {selected_joint+1}: {joint_name}")
                
                print(f"[Value] {current_qpos:.4f} rad ({math.degrees(current_qpos):.2f}°)")
                print("\nEnter command (L1-L10, R1-R10, +, -, r, c, h, q): ", end="", flush=True)
                
                cmd = input().strip().lower()
                if not cmd:
                    continue
                
                # Select joint by hand and number
                if cmd.startswith('l'):
                    try:
                        num = int(cmd[1:])
                        if 1 <= num <= self.num_dofs_per_hand:
                            selected_hand = 'left'
                            selected_joint = num - 1
                            print(f"[OK] Selected Left Hand Joint {num} ({self.left_joint_names[selected_joint]})")
                            self.publish_command(qpos_left, qpos_right)
                        else:
                            print(f"[ERROR] Left hand joint must be L1-L{self.num_dofs_per_hand}")
                    except ValueError:
                        print(f"[ERROR] Invalid format. Use L1-L{self.num_dofs_per_hand}")
                
                elif cmd.startswith('r') and len(cmd) > 1:
                    try:
                        num = int(cmd[1:])
                        if 1 <= num <= self.num_dofs_per_hand:
                            selected_hand = 'right'
                            selected_joint = num - 1
                            print(f"[OK] Selected Right Hand Joint {num} ({self.right_joint_names[selected_joint]})")
                            self.publish_command(qpos_left, qpos_right)
                        else:
                            print(f"[ERROR] Right hand joint must be R1-R{self.num_dofs_per_hand}")
                    except ValueError:
                        print(f"[ERROR] Invalid format. Use R1-R{self.num_dofs_per_hand}")
                
                # Increase
                elif cmd in ['+', '=']:
                    if selected_hand == 'left':
                        qpos_left[selected_joint] = min(qpos_left[selected_joint] + step_size, max_angle)
                        print(f"[OK] Left Joint {selected_joint+1} increased to {qpos_left[selected_joint]:.4f} rad")
                    else:
                        qpos_right[selected_joint] = min(qpos_right[selected_joint] + step_size, max_angle)
                        print(f"[OK] Right Joint {selected_joint+1} increased to {qpos_right[selected_joint]:.4f} rad")
                    self.publish_command(qpos_left, qpos_right)
                
                # Decrease
                elif cmd == '-':
                    if selected_hand == 'left':
                        qpos_left[selected_joint] = max(qpos_left[selected_joint] - step_size, -max_angle)
                        print(f"[OK] Left Joint {selected_joint+1} decreased to {qpos_left[selected_joint]:.4f} rad")
                    else:
                        qpos_right[selected_joint] = max(qpos_right[selected_joint] - step_size, -max_angle)
                        print(f"[OK] Right Joint {selected_joint+1} decreased to {qpos_right[selected_joint]:.4f} rad")
                    self.publish_command(qpos_left, qpos_right)
                
                # Reset
                elif cmd == 'r':
                    qpos_left = [0.0] * self.num_dofs_per_hand
                    qpos_right = [0.0] * self.num_dofs_per_hand
                    print("[OK] All joints reset to zero")
                    self.publish_command(qpos_left, qpos_right)
                
                # Show current state
                elif cmd == 'c':
                    print(f"\n{'Hand':<6} {'#':>3} {'Joint Name':<25} {'Angle (rad)':>12} {'Angle (°)':>10}")
                    print("-" * 65)
                    for i in range(self.num_dofs_per_hand):
                        mark = " <L" if (selected_hand == 'left' and i == selected_joint) else ""
                        print(
                            f"{'Left':<6} {i + 1:3d} {self.left_joint_names[i]:<25} "
                            f"{qpos_left[i]:12.4f} {math.degrees(qpos_left[i]):10.2f}{mark}"
                        )
                    print()
                    for i in range(self.num_dofs_per_hand):
                        mark = " <R" if (selected_hand == 'right' and i == selected_joint) else ""
                        print(
                            f"{'Right':<6} {i + 1:3d} {self.right_joint_names[i]:<25} "
                            f"{qpos_right[i]:12.4f} {math.degrees(qpos_right[i]):10.2f}{mark}"
                        )
                
                # Help
                elif cmd == 'h':
                    print("\n" + "="*70)
                    print("Command Help:")
                    print(f"  L1-L{self.num_dofs_per_hand}    : Select left hand joint")
                    print(f"  R1-R{self.num_dofs_per_hand}    : Select right hand joint")
                    print("  + / =    : Increase selected joint by", step_size, "rad")
                    print("  -        : Decrease selected joint by", step_size, "rad")
                    print("  r        : Reset all joints to zero")
                    print("  c        : Show all joint current values")
                    print("  h        : Show this help")
                    print("  q        : Quit and exit")
                    print("="*70)
                
                # Quit
                elif cmd == 'q':
                    print("[OK] Exiting...")
                    break
                
                else:
                    print(f"[ERROR] Unknown command '{cmd}'. Type 'h' for help.")
        
        except KeyboardInterrupt:
            print("\n[Interrupted by user]")
        
        finally:
            qpos_left = [0.0] * self.num_dofs_per_hand
            qpos_right = [0.0] * self.num_dofs_per_hand
            self.publish_command(qpos_left, qpos_right)
            print("[Done] All joints reset to zero and exiting keyboard control mode\n")

    def close(self):
        """Clean up ZMQ resources."""
        self.socket_left.close()
        self.socket_right.close()
        self.context.term()
        print("[Publisher] Closed")


def main():
    parser = argparse.ArgumentParser(
        description="ZMQ Publisher for Hand Teleoperator (Dual Hand Control)"
    )
    parser.add_argument(
        "--endpoint-left",
        default=DEFAULT_LEFT_ENDPOINT,
        help=f"Physical left-hand endpoint (default: {DEFAULT_LEFT_ENDPOINT})",
    )
    parser.add_argument(
        "--endpoint-right",
        default=DEFAULT_RIGHT_ENDPOINT,
        help=f"Physical right-hand endpoint (default: {DEFAULT_RIGHT_ENDPOINT})",
    )
    parser.add_argument(
        "--port-left",
        type=int,
        help="Left hand ZMQ port (creates tcp://*:<port>)",
    )
    parser.add_argument(
        "--port-right",
        type=int,
        help="Right hand ZMQ port (creates tcp://*:<port>)",
    )
    parser.add_argument(
        "--demo",
        choices=["sine", "grasp", "test", "zero", "keyboard"],
        default="test",
        help="Demo to run (default: test)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10,
        help="Duration for sine wave demo in seconds (default: 10)",
    )
    parser.add_argument(
        "--step",
        type=float,
        default=0.1,
        help="Step size for keyboard control in radians (default: 0.1)",
    )

    args = parser.parse_args()

    # Determine endpoints
    if args.port_left:
        endpoint_left = f"tcp://*:{args.port_left}"
    else:
        endpoint_left = args.endpoint_left
    
    if args.port_right:
        endpoint_right = f"tcp://*:{args.port_right}"
    else:
        endpoint_right = args.endpoint_right

    # Create publisher with dual endpoints
    pub = HandCommandPublisher(endpoint_left, endpoint_right)

    try:
        if args.demo == "sine":
            pub.publish_sine_wave(duration=args.duration, frequency=0.5, amplitude=0.5)

        elif args.demo == "grasp":
            pub.publish_grasp_motion(motion_time=2.0, repeat=3)

        elif args.demo == "test":
            pub.publish_individual_finger_test()

        elif args.demo == "zero":
            pub.publish_zero_positions(duration=args.duration)
        
        elif args.demo == "keyboard":
            pub.run_keyboard_control(step_size=args.step)

    except KeyboardInterrupt:
        print("\n[Main] Interrupted by user")

    finally:
        pub.close()


if __name__ == "__main__":
    main()
