#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
from dataclasses import dataclass
from hri_msgs.msg import IdsList, IdsMatch, NormalizedPointOfInterest2D, Skeleton2D
import functools
from math import atan2, cos, sin, sqrt
import rospy
import rostest
from sensor_msgs.msg import CameraInfo, RegionOfInterest
import sys
from typing import Dict, List, Tuple
import unittest

PKG = "hri_face_body_matcher"
TESTED_NODE = "hri_face_body_matcher"

IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080
MAX_N_PEOPLE = 2
NODE_CYCLE_SEC = 0.1
TEST_TIMEOUT_SEC = 5
TEST_MICRO_WAIT_SEC = 1e-3

POSITION_CENTER = (0.5, 0.5)
POSITION_FAR_LEFT = (0.25, 0.5)
POSITION_FAR_RIGHT = (0.75, 0.5)
POSITION_CLOSE_LEFT = (0.4, 0.5)
POSITION_CLOSE_RIGHT = (0.6, 0.5)
SIZE_STANDARD = 0.3
SIZE_SMALL = 0.1
SIZE_BIG = 0.9
DISCREPANCY_STANDARD = 0.3
DISCREPANCY_NO_MATCH = 1.0


@dataclass(frozen=True)
class Person:
    id: int = 0
    face_center_rel: Tuple[
        float, float
    ] = POSITION_CENTER  # (x,y) relative position of the person face center in the image
    face_size_rel: float = SIZE_STANDARD  # length of the face height relative to the image height
    discrepancy: float = DISCREPANCY_STANDARD  # distance between the body nose and the face center, relative to the face diagonal length
    has_face: bool = True
    has_body: bool = True
    test_matching: bool = True  # if the test is expected to find a match between the person head and body


@dataclass(frozen=True)
class TestData:
    name: str
    params: Dict


class GenericTestSequence(unittest.TestCase):
    def setUp(self):
        self.match_msgs = []
        self.camera_info_pub = rospy.Publisher(
            "/camera_info", CameraInfo, queue_size=1, latch=True
        )
        self.bodies_tracked_pub = rospy.Publisher(
            "/humans/bodies/tracked", IdsList, queue_size=1, latch=True
        )
        self.faces_tracked_pub = rospy.Publisher(
            "/humans/faces/tracked", IdsList, queue_size=1, latch=True
        )
        self.body_skeleton_pub = {}
        self.face_roi_pub = {}
        for id in range(MAX_N_PEOPLE):
            self.body_skeleton_pub[id] = rospy.Publisher(
                "/humans/bodies/body%s/skeleton2d" % id,
                Skeleton2D,
                queue_size=1,
                latch=True,
            )
            self.face_roi_pub[id] = rospy.Publisher(
                "/humans/faces/face%s/roi" % id,
                RegionOfInterest,
                queue_size=1,
                latch=True,
            )
        self.matches_sub = rospy.Subscriber(
            "/humans/candidate_matches",
            IdsMatch,
            lambda msg: self.match_msgs.append(msg),
        )

        timeout_start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            self.assertLess(
                rospy.Time.now() - timeout_start_time,
                rospy.Duration(5),
                "Timeout in waiting for node under test subscribe to topics",
            )
            if (
                self.camera_info_pub.get_num_connections()
                and self.bodies_tracked_pub.get_num_connections()
                and self.faces_tracked_pub.get_num_connections()
                and self.matches_sub.get_num_connections()
            ):
                break
            rospy.sleep(TEST_MICRO_WAIT_SEC)

        self.camera_info_pub.publish(CameraInfo(height=IMAGE_HEIGHT, width=IMAGE_WIDTH))
        self.bodies_tracked_pub.publish(IdsList())
        self.faces_tracked_pub.publish(IdsList())

        timeout_start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            self.assertLess(
                rospy.Time.now() - timeout_start_time,
                rospy.Duration(5),
                "Timeout in waiting for node under test unsubscribe to topics",
            )
            connected = True
            for id in range(MAX_N_PEOPLE):
                connected = (
                    False
                    if self.body_skeleton_pub[id].get_num_connections()
                    else connected
                )
                connected = (
                    False if self.face_roi_pub[id].get_num_connections() else connected
                )
            if connected:
                break
            rospy.sleep(TEST_MICRO_WAIT_SEC)

        self.match_msgs = []

    def tearDown(self):
        self.bodies_tracked_pub.publish(IdsList())
        self.faces_tracked_pub.publish(IdsList())

        timeout_start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            self.assertLess(
                rospy.Time.now() - timeout_start_time,
                rospy.Duration(5),
                "Timeout in waiting for node under test unsubscribe to topics",
            )
            connected = True
            for id in range(MAX_N_PEOPLE):
                connected = (
                    False
                    if self.body_skeleton_pub[id].get_num_connections()
                    else connected
                )
                connected = (
                    False if self.face_roi_pub[id].get_num_connections() else connected
                )
            if connected:
                break
            rospy.sleep(TEST_MICRO_WAIT_SEC)

        self.camera_info_pub = None
        self.bodies_tracked_pub = None
        self.faces_tracked_pub = None
        self.body_skeleton_pub = None
        self.face_roi_pub = None
        self.matches_sub = None

    def _test(self, name, people: List[Person]):
        rospy.logdebug(f"Running test_{name}")
        bodies_tracked_msg = IdsList()
        faces_tracked_msg = IdsList()
        body_skeleton_msgs = {}
        face_roi_msgs = {}
        expected_match_msgs = []
        face_ratio = 1.62

        for id, person in enumerate(people):
            if person.has_face:
                face_center_x = person.face_center_rel[0] * IMAGE_WIDTH
                face_center_y = person.face_center_rel[1] * IMAGE_HEIGHT
                face_height = int(person.face_size_rel * IMAGE_HEIGHT)
                face_width = int(face_height / face_ratio)
                face_min_x = int(face_center_x - (face_width / 2))
                face_max_x = int(face_center_x + (face_width / 2))
                face_min_y = int(face_center_y - (face_height / 2))
                face_max_y = int(face_center_y + (face_height / 2))
                self.assertGreaterEqual(
                    face_min_x, 0, "Computed face RoI is out of bounds"
                )
                self.assertGreaterEqual(
                    face_min_y, 0, "Computed face RoI is out of bounds"
                )
                self.assertLessEqual(
                    face_max_x, IMAGE_WIDTH, "Computed face RoI is out of bounds"
                )
                self.assertLessEqual(
                    face_max_y, IMAGE_HEIGHT, "Computed face RoI is out of bounds"
                )

                faces_tracked_msg.ids.append(f"face{id}")
                face_roi_msgs[id] = RegionOfInterest(
                    x_offset=face_min_x,
                    y_offset=face_min_y,
                    height=face_height,
                    width=face_width,
                )
                rospy.logdebug(
                    f"Publish face{id} with RoI x_offset={face_min_x}, y_offset={face_min_y}, height={face_height}, width={face_width}"
                )

            if person.has_body:
                if person.has_face:
                    # shift the node position from the image center always towards the image center
                    image_center_x = IMAGE_WIDTH / 2
                    image_center_y = IMAGE_HEIGHT / 2
                    face_to_image_center_angle = atan2(
                        (image_center_y - face_center_y),
                        (image_center_x - face_center_x),
                    )
                    face_diag_length = sqrt(face_height ** 2 + face_width ** 2)
                    nose_x = (
                        face_center_x
                        + cos(face_to_image_center_angle)
                        * face_diag_length
                        * person.discrepancy
                    )
                    nose_y = (
                        face_center_y
                        + sin(face_to_image_center_angle)
                        * face_diag_length
                        * person.discrepancy
                    )
                    nose_x_rel = nose_x / IMAGE_WIDTH
                    nose_y_rel = nose_y / IMAGE_HEIGHT
                else:
                    nose_x_rel = person.face_center_rel[0]
                    nose_y_rel = person.face_center_rel[1]
                self.assertGreaterEqual(
                    nose_x_rel, 0.0, "Computed body nose position is out of bounds"
                )
                self.assertLessEqual(
                    nose_x_rel, 1.0, "Computed body nose position is out of bounds"
                )
                self.assertGreaterEqual(
                    nose_y_rel, 0.0, "Computed body nose position is out of bounds"
                )
                self.assertLessEqual(
                    nose_y_rel, 1.0, "Computed body nose position is out of bounds"
                )

                bodies_tracked_msg.ids.append(f"body{id}")
                body_skeleton = Skeleton2D()
                body_skeleton.skeleton.insert(
                    Skeleton2D.NOSE,
                    NormalizedPointOfInterest2D(x=nose_x_rel, y=nose_y_rel),
                )
                body_skeleton_msgs[id] = body_skeleton
                rospy.logdebug(
                    f"Publish body{id} with nose relative position x={nose_x_rel}, y={nose_y_rel}"
                )

            if person.test_matching:
                msg = IdsMatch(id1=f"body{id}", id1_type=IdsMatch.BODY)
                msg.id1 = f"body{id}"
                msg.id1_type = IdsMatch.BODY
                msg.id2 = f"face{id}"
                msg.id2_type = IdsMatch.FACE
                expected_match_msgs.append(
                    IdsMatch(
                        id1=f"body{id}",
                        id1_type=IdsMatch.BODY,
                        id2=f"face{id}",
                        id2_type=IdsMatch.FACE,
                    )
                )
                rospy.logdebug(f"Expected matching body{id} with face{id}")

        self.bodies_tracked_pub.publish(bodies_tracked_msg)
        self.faces_tracked_pub.publish(faces_tracked_msg)

        timeout_start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            self.assertLess(
                rospy.Time.now() - timeout_start_time,
                rospy.Duration(5),
                "Timeout in waiting for node under test subscribe to topics",
            )
            connected = True
            for id, person in enumerate(people):
                if person.has_body:
                    connected = (
                        False
                        if not self.body_skeleton_pub[id].get_num_connections()
                        else connected
                    )
                if person.has_face:
                    connected = (
                        False
                        if not self.face_roi_pub[id].get_num_connections()
                        else connected
                    )
            if connected:
                break
            rospy.sleep(TEST_MICRO_WAIT_SEC)

        for id, msg in body_skeleton_msgs.items():
            self.body_skeleton_pub[id].publish(msg)
        for id, msg in face_roi_msgs.items():
            self.face_roi_pub[id].publish(msg)

        rospy.sleep(2 * NODE_CYCLE_SEC)

        for e_msg in expected_match_msgs:
            rospy.logdebug(f"Expected matching message {e_msg}")
            self.assertIn(
                (e_msg.id1, e_msg.id1_type, e_msg.id2, e_msg.id2_type),
                [
                    (m_msg.id1, m_msg.id1_type, m_msg.id2, m_msg.id2_type)
                    for m_msg in self.match_msgs
                ],
                "Failed to find an expected match",
            )
        for m_msg in self.match_msgs:
            rospy.logdebug(f"Received matching message {m_msg}")
            self.assertIn(
                (m_msg.id1, m_msg.id1_type, m_msg.id2, m_msg.id2_type),
                [
                    (e_msg.id1, e_msg.id1_type, e_msg.id2, e_msg.id2_type)
                    for e_msg in expected_match_msgs
                ],
                f"Found an unexpected match with confidence {m_msg.confidence}",
            )


class TestSequenceMeta(type):
    def __new__(mcs, name, bases, dct):
        def gen_test(name, params):
            def test(self):
                self._test(name, **params)

            return test

        test_sequence_data = dct["test_sequence_data"]
        for test in test_sequence_data:
            dct[f"test_{test.name}"] = gen_test(test.name, test.params)
        return type.__new__(mcs, name, bases, dct)


class TestSequenceDefault(GenericTestSequence, metaclass=TestSequenceMeta):
    test_sequence_data = [
        TestData(name="one_person", params={"people": [Person()]}),
        TestData(
            name="one_person_no_match",
            params={
                "people": [
                    Person(discrepancy=DISCREPANCY_NO_MATCH, test_matching=False)
                ]
            },
        ),
        TestData(
            name="one_small_person",
            params={"people": [Person(face_size_rel=SIZE_SMALL)]},
        ),
        TestData(
            name="one_big_person", params={"people": [Person(face_size_rel=SIZE_BIG)]}
        ),
        TestData(
            name="one_person_no_face",
            params={"people": [Person(has_face=False, test_matching=False)]},
        ),
        TestData(
            name="one_person_no_body",
            params={"people": [Person(has_body=False, test_matching=False)]},
        ),
        TestData(
            name="two_people",
            params={
                "people": [
                    Person(face_center_rel=POSITION_FAR_LEFT),
                    Person(face_center_rel=POSITION_FAR_RIGHT, id=1),
                ]
            },
        ),
        TestData(
            name="two_people_no_match",
            params={
                "people": [
                    Person(
                        face_center_rel=POSITION_FAR_LEFT,
                        discrepancy=DISCREPANCY_NO_MATCH,
                        test_matching=False,
                    ),
                    Person(
                        face_center_rel=POSITION_FAR_RIGHT,
                        discrepancy=DISCREPANCY_NO_MATCH,
                        test_matching=False,
                        id=1,
                    ),
                ]
            },
        ),
        TestData(
            name="two_close_people",
            params={
                "people": [
                    Person(face_center_rel=POSITION_CLOSE_LEFT),
                    Person(face_center_rel=POSITION_CLOSE_RIGHT, id=1),
                ]
            },
        ),
        TestData(
            name="two_people_one_without_face",
            params={
                "people": [
                    Person(face_center_rel=POSITION_FAR_LEFT),
                    Person(
                        face_center_rel=POSITION_FAR_RIGHT,
                        has_face=False,
                        test_matching=False,
                        id=1,
                    ),
                ]
            },
        ),
        TestData(
            name="two_people_one_without_body",
            params={
                "people": [
                    Person(
                        face_center_rel=POSITION_FAR_LEFT,
                        has_face=False,
                        test_matching=False,
                    ),
                    Person(face_center_rel=POSITION_FAR_RIGHT, id=1),
                ]
            },
        ),
    ]


class TestSequenceHighConfidenceScaling(
    GenericTestSequence, metaclass=TestSequenceMeta
):
    test_sequence_data = [
        TestData(name="one_person", params={"people": [Person(test_matching=False)]})
    ]


class TestSequenceHighThreshold(GenericTestSequence, metaclass=TestSequenceMeta):
    test_sequence_data = [
        TestData(name="one_person", params={"people": [Person(test_matching=False)]})
    ]


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--test_case", nargs=1)
    args, unknown = parser.parse_known_args()
    test_case = args.test_case[0]

    rospy.init_node("tester_hri_face_body_matcher", log_level=rospy.DEBUG)
    if test_case == "default":
        test_class = TestSequenceDefault
    elif test_case == "high_confidence_scaling":
        test_class = TestSequenceHighConfidenceScaling
    elif test_case == "high_confidence_threshold":
        test_class = TestSequenceHighThreshold
    else:
        rospy.logerr(f"Test case {test_case} not recognized.")

    rostest.rosrun(PKG, test_case, test_class, sys.argv)