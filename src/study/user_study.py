#!/usr/bin/env python3
import rospy
import numpy as np
import random
from yaml import load, dump, safe_load
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper
from baselines import TrajectoryRunner, positions, rt_comoto
from os.path import abspath, isfile, isdir, join
from os import getcwd, listdir
from pygame import mixer
from requests import post
from time import sleep
import sys
from control_msgs.msg import GripperCommandActionGoal

from record_data import DataRecorder


class UserStudy():
    def __init__(self):
        self.num_bolts = 7
        self.num_participants = 20
        self.participant_id_num_sig_figs = 3
        self.goals = positions.posns
        self.home = positions.home
        self.post_link = "http://127.0.0.1:5000/get-which-img"
        self.robot_delay = 4 # in seconds
        self.robot_timeout = 60 # in seconds

        self.runner = TrajectoryRunner()
        mixer.init()
        self.startTrialBeep = mixer.Sound(join(getcwd(), 'assets', 'chime.wav'))
        self.robotInspectBeep = mixer.Sound(join(getcwd(), 'assets', 'inspect.wav'))
        
        gripper_pub = rospy.Publisher("/right_arm/right_arm_robotiq_2f_85_gripper_controller/gripper_cmd/goal", GripperCommandActionGoal, queue_size=1)
        rospy.sleep(0.5)
        gripper_msg = GripperCommandActionGoal()
        gripper_msg.goal.command.position = 0.75
        gripper_msg.goal.command.max_effort = 0.01
        gripper_pub.publish(gripper_msg)
        rospy.sleep(1)
        gripper_pub.unregister()

        # Algorithms
        self.C = "comoto"
        self.S = "sa"
        self.R = "rrt"
        self.L = "leg"
        self.N = "nominal"

        # Balanced Latin Square Generation
        self.bls_1 = [self.C, self.S, self.R, self.L]
        self.bls_2 = [self.S, self.L, self.C, self.R]
        self.bls_3 = [self.L, self.R, self.S, self.C]
        self.bls_4 = [self.R, self.C, self.L, self.S]
        
        self.restore = len(sys.argv) > 1 and sys.argv[1] == 'restore'
        if not self.restore:
            result = input('Restore from last run? [y/N]: ')
            if result.lower().strip() == 'y':
                self.restore = True
        print("Restore: ", self.restore)

        # Load or generate Participant_ID_Study_Sequence
        # To generate new Participant_ID_Study_Sequence, delete Participant_ID_Study_Sequence.yaml
        if isfile(getcwd() + '/Participant_ID_Study_Sequence.yaml'):
            with open('Participant_ID_Study_Sequence.yaml', 'r') as f:
                self.participant_id_study_sequence = safe_load(f)
        else:
            self.participant_id_study_sequence = {}
            self.create_yaml_dict()

        print("Loaded Participant ID - Study Sequence dictionary.")

        # Load or generate HR_sequences
        # To generate new HR_sequences, delete HR_Sequences.yaml
        if isfile(getcwd() + '/HR_Sequences.yaml'):
            f = open('HR_Sequences.yaml', 'r')
            self.HR_sequences = load(f, Loader=Loader)
            f.close()
        else:
            self.HR_sequences = {}
            self.gen_random_HR_sequences()

        print("Loaded Human-Robot Sequences.")
        for i in range(4):
            print(f"Trial {i + 1} => Human: {self.HR_sequences[i + 1][0]}, Robot: {self.HR_sequences[i + 1][1]}")

        saved_subjects = []

        for entry in listdir(getcwd() + "/data/subjects"):
            if isdir(join(getcwd() + "/data/subjects", entry)):
                saved_subjects.append(str(entry).zfill(self.participant_id_num_sig_figs))

        saved_subjects = sorted(saved_subjects)

        id = 1
        if len(saved_subjects) > 0:
            id = int(saved_subjects[-1]) + 1
            if self.restore:
                id -= 1
        self.participant_id = str(id).zfill(self.participant_id_num_sig_figs)
        print("Participant ID = ", self.participant_id)

        self.participant_study_sequence = self.participant_id_study_sequence[self.participant_id]
        print("Participant Study Sequence = ", self.participant_study_sequence)

        self.display_img = {'which_img': 'wait.png'}
        post(self.post_link, json=self.display_img)

        print("Initializing robot to home position")
        self.runner.run_trajectory(self.N, self.home)
        print("Robot is home")

        self.run_study()

    def choose_bls(self, num):
        if num == 1:
            return self.bls_1
        elif num == 2:
            return self.bls_2
        elif num == 3:
            return self.bls_3
        elif num == 4:
            return self.bls_4
        else:
            return None

    def create_yaml_dict(self):
        print("Generating new Participant ID - Study Sequence dictionary...")

        for id in range(1, self.num_participants):
            participant_id = str(id).zfill(self.participant_id_num_sig_figs)
            participant_study_sequence = self.choose_bls((id % 4) + 1)
            self.participant_id_study_sequence[participant_id] = participant_study_sequence


        with open('Participant_ID_Study_Sequence.yaml', 'w') as f:
            dump(self.participant_id_study_sequence, f, default_flow_style=False)

        print("Saved generated Participant ID - Study Sequence dictionary to 'Participant_ID_Study_Sequence.yaml'")

    def gen_random_HR_sequences(self):
        print("Generating new HR Sequences...")
        all_sequences = []

        for i in range(4):
            human_sequence = np.array(random.sample(range(1, self.num_bolts + 1), self.num_bolts))

            while (tuple(human_sequence) in all_sequences):
                human_sequence = np.array(random.sample(range(1, self.num_bolts + 1), self.num_bolts))

            print(f"Human sequence: {human_sequence}")
            key = input("Do you approve this human sequence? (y/n): ")
            while (not key == 'y'):
                human_sequence = np.array(random.sample(range(1, self.num_bolts + 1), self.num_bolts))

                while (tuple(human_sequence) in all_sequences):
                    human_sequence = np.array(random.sample(range(1, self.num_bolts + 1), self.num_bolts))

                print(f"Human sequence: {human_sequence}")
                key = input("Do you approve this human sequence? (y/n): ")

            all_sequences.append(tuple(human_sequence))
            print()
            print()

            robot_sequence = np.array(random.sample(range(1, self.num_bolts + 1), self.num_bolts))
            while ((robot_sequence == human_sequence).any() or (tuple(robot_sequence) in all_sequences)):
                robot_sequence = np.array(random.sample(range(1, self.num_bolts + 1), self.num_bolts))

            print(f"Robot sequence: {robot_sequence}")
            key = input("Do you approve this robot sequence? (y/n): ")
            while (not key == 'y'):
                robot_sequence = np.array(random.sample(range(1, self.num_bolts + 1), self.num_bolts))

                while ((robot_sequence == human_sequence).any() or (tuple(robot_sequence) in all_sequences)):
                    robot_sequence = np.array(random.sample(range(1, self.num_bolts + 1), self.num_bolts))

                print(f"Robot sequence: {robot_sequence}")
                key = input("Do you approve this robot sequence? (y/n): ")

            all_sequences.append(tuple(robot_sequence))

            # HR_sequences[trial number] = (human sequence of bolts, robot sequence of bolts)
            self.HR_sequences[i + 1] = (human_sequence, robot_sequence)
            print(f"Trial {i + 1} => Human: {self.HR_sequences[i + 1][0]}, Robot: {self.HR_sequences[i + 1][1]}")
            print("==========================================================")

        with open('HR_Sequences.yaml', 'w') as f:
            dump(self.HR_sequences, f, default_flow_style=False)

        print("Saved generated HR Sequences to 'HR_Sequences.yaml'")

    def run_study(self):
        for i in range(len(self.participant_study_sequence)):
            if self.participant_study_sequence[i] == self.N:
                self.participant_study_sequence[i] = self.L
        print("Warm-starting CoMOTO...")
        warm_start_client = rt_comoto.TrajectoryManager(self.goals[0], warm_start=True)
        key = input("Press Enter to continue to Trial 0 (Test Run), s to skip: ")
        recorder = DataRecorder(restore=self.restore)
        if not key == 's':

            # Test run (Trial 0) => R = [1, 2] , H = [5, 3]
            print(f"Trial 0: {self.N}")
            print("----------------------------------------------------------")
            human_sequence = [5, 3]
            robot_sequence = [1, 2]
            print(f"Human sequence: {human_sequence}")
            print(f"Robot sequence: {robot_sequence}")

            # Display current human sequence
            self.display_img['which_img'] = f"T0_human_bolt_ordering.png"
            post(self.post_link, json=self.display_img)
            self.startTrialBeep.play()

            # Initiate current robot sequence
            for current_goal_idx in robot_sequence:
                print(f"Robot is navigating to bolt {current_goal_idx}")
                current_goal = self.goals[current_goal_idx - 1, :]
                success = self.runner.run_trajectory(self.N, current_goal, timeout = self.robot_timeout)
                if success:
                    print(f"Robot successfully navigated to bolt {current_goal_idx}")
                    sleep(self.robot_delay)
                    self.robotInspectBeep.play()

            print("Returning robot to home position")
            self.runner.run_trajectory(self.N, self.home)
            print("Robot is home")

            # Press Enter once human is done with sequence
            input("Press Enter to show wait: ")

            # Display 'wait' until ready for next sequence
            self.display_img['which_img'] = f"wait.png"
            post(self.post_link, json=self.display_img)
            print("Wait screen is displayed.")

            print("==========================================================")


        # Trials 1 - 4
        for trial_num in range(1, 5):

            if self.participant_study_sequence[trial_num - 1] == self.C:
                warm_start_client.teardown()

            # Press Enter to start next trial
            key = input(f"Press Enter to continue to Trial {trial_num} ({self.participant_study_sequence[trial_num - 1]}), s to skip: ")
             
            if not key == 's':

                print(f"Trial {trial_num}: {self.participant_study_sequence[trial_num - 1]}")
                print("----------------------------------------------------------")
                human_sequence = self.HR_sequences[trial_num][0]
                robot_sequence = self.HR_sequences[trial_num][1]
                print(f"Human sequence: {human_sequence}")
                print(f"Robot sequence: {robot_sequence}")

                # Display current human sequence
                self.display_img['which_img'] = f"T{trial_num}_human_bolt_ordering.png"
                post(self.post_link, json=self.display_img)
                self.startTrialBeep.play()

                # Initiate current robot sequence
                for current_goal_idx in robot_sequence:
                    print(f"Robot is navigating to bolt {current_goal_idx}")
                    current_goal = self.goals[current_goal_idx - 1, :]
                    current_method = self.participant_study_sequence[trial_num - 1]
                    try:
                        success = self.runner.run_trajectory(current_method, current_goal, timeout=self.robot_timeout)
                        if success:
                            print(f"Robot successfully navigated to bolt {current_goal_idx}")
                            sleep(self.robot_delay)
                            self.robotInspectBeep.play()
                    except Exception as e:
                        print("Encountered an error!", repr(e))
                        print("Failed to drive robot to goal. Will continue with nominal and resume with original method next goal")
                        self.runner.run_trajectory(self.N, current_goal, timeout=self.robot_timeout)



                print("Returning robot to home position")
                self.runner.run_trajectory(self.N, self.home)
                print("Robot is home")

                # Press Enter once human is done with sequence
                input("Press Enter to show wait: ")

                # Display 'wait' until ready for next sequence
                self.display_img['which_img'] = "wait.png"
                post(self.post_link, json=self.display_img)
                print("Wait screen is displayed.")

                print("==========================================================")

        recorder.stop()

if __name__ == "__main__":
    rospy.init_node("comoto_user_study", anonymous=True)
    us = UserStudy()
    rospy.signal_shutdown("Done..")
    exit(0)
