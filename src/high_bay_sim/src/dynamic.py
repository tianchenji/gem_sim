import math

import rospy

from std_msgs.msg import Int32, Float64
from geometry_msgs.msg import Twist, Pose, Point
from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelStates, LinkStates, LinkState, ModelState


def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [roll, pitch, yaw]


class bicycleModel():

    def __init__(self):

        self.length = 0.4

        self.waypointSub = rospy.Subscriber("/gem/waypoint", Point, self.__waypointHandler, queue_size=1)

        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)

    def getModelState(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            modelState = serviceResponse(model_name='polaris')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
        return modelState


    def rearWheelModel(self, ackermannCmd):
        currentModelState = self.getModelState()

        if not currentModelState.success:
            return

        velocity = ackermannCmd.speed
        steeringAngle = ackermannCmd.steering_angle

        euler = quaternion_to_euler(currentModelState.pose.orientation.x,
                                    currentModelState.pose.orientation.y,
                                    currentModelState.pose.orientation.z,
                                    currentModelState.pose.orientation.w)

        xVelocity = velocity * math.cos(euler[2])
        yVelocity = velocity * math.sin(euler[2])
        thetaVelocity = (velocity/self.length) * (math.tan(steeringAngle))

        return [xVelocity, yVelocity, thetaVelocity]

    def rearWheelFeedback(self, currentPose, targetPose):

        k1 = 1
        k2 = 10
        k3 = 1
        targetV = 1
        targetw = 1

        currentEuler = quaternion_to_euler(currentPose.orientation.x,
                                           currentPose.orientation.y,
                                           currentPose.orientation.z,
                                           currentPose.orientation.w)

        targetEuler = quaternion_to_euler(targetPose.orientation.x,
                                          targetPose.orientation.y,
                                          targetPose.orientation.z,
                                          targetPose.orientation.w)

        #compute errors
        xError = ((targetPose.position.x - currentPose.position.x) * math.cos(currentEuler[2])) + ((targetPose.position.y - currentPose.position.y) * math.sin(currentEuler[2]))
        yError = ((targetPose.position.x - currentPose.position.x) * math.sin(currentEuler[2]) * -1) + ((targetPose.position.y - currentPose.position.y) * math.cos(currentEuler[2]))
        thetaError = targetEuler[2] - currentEuler[2]

        print(xError, yError, thetaError)

        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = (targetV * math.cos(thetaError)) + (k1 * xError)
        newAckermannCmd.steering_angle = (targetw) + ((targetV)*((k2*yError) + (k3*math.sin(thetaError))))

        print(newAckermannCmd)
        return newAckermannCmd




    def __waypointHandler(self, data):
        print(data)



if __name__ == "__main__":
    rospy.init_node("model_dynamics")

    model = bicycleModel()
    control = AckermannDrive()

    target = Pose()
    target.position.x = 0
    target.position.y = 0
    target.position.z = 0
    target.orientation.x = 0
    target.orientation.y = 0
    target.orientation.z = 0
    target.orientation.w = 0


    rate = rospy.Rate(100)  # 100 Hz
    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state

        state =  model.getModelState()
        if not state.success:
            continue


        control = model.rearWheelFeedback(state.pose, target)
        values = model.rearWheelModel(control)

        newState = ModelState()
        newState.model_name = 'polaris'
        newState.pose = state.pose
        newState.twist.linear.x =values[0]
        newState.twist.linear.y = values[1]
        newState.twist.angular.z = values[2]
        model.modelStatePub.publish(newState)

    rospy.spin()
