import unittest
import rclpy
import computer_node as cn

from unittest import mock
from geometry_msgs.msg import Twist

from rclpy.node import Node
from unittest import TestCase



class computer_nodeTest(TestCase):

    def test_init(self):
        rclpy.init(args=None)

    def test_updateVelocity_simpleForwardAcceleration(self):

        #Arrange
        expectedAction = Twist()
        comp_node = cn.Computer_Node()
        comp_node.pub_action = mock.MagicMock()

        comp_node.desiredMovement.x = 0.1
        comp_node.currentMovement.x = 0.0
        expectedAction.linear.x = 0.05
        expectedAction.angular.z = 0.0
        #Act
        comp_node.updateVelocity()

        #Assert
        mock.Mock.assert_called_once_with(comp_node.pub_action.publish, expectedAction)
        assert comp_node.pub_action.publish.called

        #Cleanup
        comp_node.destroy_node()

    def test_updateVelocity_simpleDeacceleration(self):

        #Arrange
        expectedAction = Twist()
        comp_node = cn.Computer_Node()
        comp_node.pub_action = mock.MagicMock()

        comp_node.desiredMovement.x = 0.0
        comp_node.currentMovement.x = 0.1
        expectedAction.linear.x = 0.05
        expectedAction.angular.z = 0.0
        #Act
        comp_node.updateVelocity()

        #Assert
        mock.Mock.assert_called_once_with(comp_node.pub_action.publish, expectedAction)
        assert comp_node.pub_action.publish.called

        #Cleanup
        comp_node.destroy_node()

    def test_updateVelocity_simpleSmallForwardAcceleration(self):

        #Arrange
        expectedAction = Twist()
        comp_node = cn.Computer_Node()
        comp_node.pub_action = mock.MagicMock()

        comp_node.desiredMovement.x = 0.02
        comp_node.currentMovement.x = 0.0
        expectedAction.linear.x = 0.02
        expectedAction.angular.z = 0.0
        #Act
        comp_node.updateVelocity()

        #Assert
        mock.Mock.assert_called_once_with(comp_node.pub_action.publish, expectedAction)
        assert comp_node.pub_action.publish.called

        #Cleanup
        comp_node.destroy_node()

    def test_updateVelocity_simpleSmallDeacceleration(self):

        #Arrange
        expectedAction = Twist()
        comp_node = cn.Computer_Node()
        comp_node.pub_action = mock.MagicMock()

        comp_node.desiredMovement.x = 0.08
        comp_node.currentMovement.x = 0.1
        expectedAction.linear.x = 0.08
        expectedAction.angular.z = 0.0
        #Act
        comp_node.updateVelocity()

        #Assert
        mock.Mock.assert_called_once_with(comp_node.pub_action.publish, expectedAction)
        assert comp_node.pub_action.publish.called

        #Cleanup
        comp_node.destroy_node()

    def test_updateVelocity_simpleSmallDeaccelTurn(self):

        #Arrange
        expectedAction = Twist()
        comp_node = cn.Computer_Node()
        comp_node.pub_action = mock.MagicMock()

        comp_node.desiredMovement.z = 0.08
        comp_node.currentMovement.z = 0.1
        expectedAction.linear.x = 0.0
        expectedAction.angular.z = 0.08
        #Act
        comp_node.updateVelocity()

        #Assert
        mock.Mock.assert_called_once_with(comp_node.pub_action.publish, expectedAction)
        assert comp_node.pub_action.publish.called

        #Cleanup
        comp_node.destroy_node()

    def test_updateVelocity_simpleSmallTurn(self):

        #Arrange
        expectedAction = Twist()
        comp_node = cn.Computer_Node()
        comp_node.pub_action = mock.MagicMock()

        comp_node.desiredMovement.z = 0.5
        comp_node.currentMovement.z = 0.0
        expectedAction.linear.x = 0.0
        expectedAction.angular.z = 0.5
        #Act
        comp_node.updateVelocity()

        #Assert
        mock.Mock.assert_called_once_with(comp_node.pub_action.publish, expectedAction)
        assert comp_node.pub_action.publish.called

        #Cleanup
        comp_node.destroy_node()


    def test_updateVelocity_simpleDeaccelTurn(self):

        #Arrange
        expectedAction = Twist()
        comp_node = cn.Computer_Node()
        comp_node.pub_action = mock.MagicMock()

        comp_node.desiredMovement.z = 0.0
        comp_node.currentMovement.z = 1.3
        expectedAction.linear.x = 0.0
        expectedAction.angular.z = 0.30000000000000004
        #Act
        comp_node.updateVelocity()

        #Assert
        mock.Mock.assert_called_once_with(comp_node.pub_action.publish, expectedAction)
        assert comp_node.pub_action.publish.called

        #Cleanup
        comp_node.destroy_node()

    def test_updateVelocity_simpleTurn(self):

        #Arrange
        expectedAction = Twist()
        comp_node = cn.Computer_Node()
        comp_node.pub_action = mock.MagicMock()

        comp_node.desiredMovement.z = 1.5
        comp_node.currentMovement.z = 0.0
        expectedAction.linear.x = 0.0
        expectedAction.angular.z = 1.0
        #Act
        comp_node.updateVelocity()

        #Assert
        mock.Mock.assert_called_once_with(comp_node.pub_action.publish, expectedAction)
        assert comp_node.pub_action.publish.called

        #Cleanup
        comp_node.destroy_node()


    def test_updateVelocity_MixedMovement(self):

        #Arrange
        expectedAction = Twist()
        comp_node = cn.Computer_Node()
        comp_node.pub_action = mock.MagicMock()

        comp_node.desiredMovement.x = 0.1
        comp_node.currentMovement.x = 0.0
        comp_node.desiredMovement.z = 0.08
        comp_node.currentMovement.z = 0.1
        expectedAction.linear.x = 0.05
        expectedAction.angular.z = 0.08

        #Act
        comp_node.updateVelocity()

        #Assert
        mock.Mock.assert_called_once_with(comp_node.pub_action.publish, expectedAction)
        assert comp_node.pub_action.publish.called

        #Cleanup
        comp_node.destroy_node()

if __name__ == '__main__':
    unittest.main()