#! /usr/bin/python

from contextlib import contextmanager
import rospy
from panda_gym.rospy_utils import service
from std_srvs.srv import Empty
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest, GetPhysicsProperties
from geometry_msgs.msg import Vector3


class Gazebo(object):
    """The class that encapsulated services and parameters to work with Gazebo
    """

    def __init__(self):
        timeout = 2
        self._unpause_srv = service(
            '/gazebo/unpause_physics', Empty, timeout)
        self._pause_srv = service(
            '/gazebo/pause_physics', Empty, timeout)
        self._reset_sim_srv = service(
            '/gazebo/reset_simulation', Empty, timeout)
        self._reset_world_srv = service(
            '/gazebo/reset_world', Empty, timeout)
        self._set_physics_srv = service(
            '/gazebo/set_physics_properties', SetPhysicsProperties, timeout)
        self._get_physics_srv = service(
            '/gazebo/get_physics_properties', GetPhysicsProperties, timeout)


    @property
    def time_step(self):
        """Get or set the current time_step. Setting the time_step to a new value
        will reconfigure the gazebo automatically.
        """
        self._get_physics()
        return self._time_step

    @time_step.setter
    def time_step(self, value):
        self._set_physics(time_step=value)

    @property
    def max_update_rate(self):
        """Get or set the current max_update_rate. Setting the max_update_rate
        to a new value will reconfigure the gazebo automatically.
        """
        self._get_physics()
        return self._max_update_rate

    @max_update_rate.setter
    def max_update_rate(self, value):
        self._set_physics(max_update_rate=value)

    @property
    def gravity_x(self):
        """Get or set the current gravity_x. Setting the gravity_x to a new value
        will reconfigure the gazebo automatically.
        """
        self._get_physics()
        return self._gravity_x

    @gravity_x.setter
    def gravity_x(self, value):
        self._set_physics(gravity_x=value)

    @property
    def gravity_y(self):
        """Get or set the current gravity_y. Setting the gravity_y to a new value
        will reconfigure the gazebo automatically.
        """
        self._get_physics()
        return self._gravity_y

    @gravity_y.setter
    def gravity_y(self, value):
        self._set_physics(gravity_y=value)

    @property
    def gravity_z(self):
        """Get or set the current gravity_z. Setting the gravity_z to a new value
        will reconfigure the gazebo automatically.
        """
        self._get_physics()
        return self._gravity_z

    @gravity_z.setter
    def gravity_z(self, value):
        self._set_physics(gravity_z=value)

    @property
    def auto_disable_bodies(self):
        """Get or set the current auto_disable_bodies. Setting the
        auto_disable_bodies to a new value will reconfigure the gazebo
        automatically.
        """
        self._get_physics()
        return self._auto_disable_bodies

    @auto_disable_bodies.setter
    def auto_disable_bodies(self, value):
        self._set_physics(auto_disable_bodies=value)

    @property
    def sor_pgs_precon_iters(self):
        """Get or set the current sor_pgs_precon_iters. Setting the
        sor_pgs_precon_iters to a new value will reconfigure the gazebo
        automatically.
        """
        self._get_physics()
        return self._sor_pgs_precon_iters

    @sor_pgs_precon_iters.setter
    def sor_pgs_precon_iters(self, value):
        self._set_physics(sor_pgs_precon_iters=value)

    @property
    def sor_pgs_iters(self):
        """Get or set the current sor_pgs_iters. Setting the sor_pgs_iters to a
        new value will reconfigure the gazebo automatically.
        """
        self._get_physics()
        return self._sor_pgs_iters

    @sor_pgs_iters.setter
    def sor_pgs_iters(self, value):
        self._set_physics(sor_pgs_iters=value)

    @property
    def sor_pgs_w(self):
        """Get or set the current sor_pgs_w. Setting the sor_pgs_w to a new
        value will reconfigure the gazebo automatically.
        """
        self._get_physics()
        return self._sor_pgs_w

    @sor_pgs_w.setter
    def sor_pgs_w(self, value):
        self._set_physics(sor_pgs_w=value)

    @property
    def sor_pgs_rms_error_tol(self):
        """Get or set the current sor_pgs_rms_error_tol. Setting the
        sor_pgs_rms_error_tol to a new value will reconfigure the gazebo
        automatically.
        """
        self._get_physics()
        return self._sor_pgs_rms_error_tol

    @sor_pgs_rms_error_tol.setter
    def sor_pgs_rms_error_tol(self, value):
        self._set_physics(sor_pgs_rms_error_tol=value)

    @property
    def contact_surface_layer(self):
        """Get or set the current contact_surface_layer. Setting the
        contact_surface_layer to a new value will reconfigure the gazebo
        automatically.
        """
        self._get_physics()
        return self._contact_surface_layer

    @contact_surface_layer.setter
    def contact_surface_layer(self, value):
        self._set_physics(contact_surface_layer=value)

    @property
    def contact_max_correcting_vel(self):
        """Get or set the current contact_max_correcting_vel. Setting the
        contact_max_correcting_vel to a new value will reconfigure the gazebo
        automatically.
        """
        self._get_physics()
        return self._contact_max_correcting_vel

    @contact_max_correcting_vel.setter
    def contact_max_correcting_vel(self, value):
        self._set_physics(contact_max_correcting_vel=value)

    @property
    def cfm(self):
        """Get or set the current cfm. Setting the cfm to a new value
        will reconfigure the gazebo automatically.
        """
        self._get_physics()
        return self._cfm

    @cfm.setter
    def cfm(self, value):
        self._set_physics(cfm=value)

    @property
    def erp(self):
        """Get or set the current erp. Setting the erp to a new value
        will reconfigure the gazebo automatically.
        """
        self._get_physics()
        return self._erp

    @erp.setter
    def erp(self, value):
        self._set_physics(erp=value)

    @property
    def max_contacts(self):
        """Get or set the current max_contacts. Setting the max_contacts to a
        new value will reconfigure the gazebo automatically.
        """
        self._get_physics()
        return self._max_contacts

    @max_contacts.setter
    def max_contacts(self, value):
        self._set_physics(max_contacts=value)

    def pause(self):
        """Pauses the simulation

        Raises:
            rospy.ServiceException if pausing fails
        """
        rospy.logdebug("Pausing the simulation")
        try:
            r = self._pause_srv()
        except rospy.ServiceException as e:
            rospy.logerr("Simulation pausing service call failed")
            raise e

    def unpause(self):
        """Unpause the simulation

        Raises:
            rospy.ServiceException if unpausing fails
        """
        rospy.logdebug("Unpausing the simulation")
        try:
            self._unpause_srv()
        except rospy.ServiceException as e:
            rospy.logerr("Simulation unpausing service call failed")
            raise e

    def reset_sim(self):
        """Reset the simulation

        Raises:
            rospy.ServiceException if reseting fails
        """
        rospy.logdebug("Reseting the simulation")
        try:
            self._reset_sim_srv()
        except rospy.ServiceException as e:
            rospy.logerr("Simulation reset service call failed")
            raise e

    def reset_world(self):
        """Reset the world

        Raises:
            rospy.ServiceException if reseting fails
        """
        try:
            self._reset_world_srv()
        except rospy.ServiceException as e:
            rospy.logerr("Simulation reset service call failed")
            raise e

    def _get_physics(self):
        """Prive method that update the physics private attributes
        """
        physics = self._get_physics_srv()
        rospy.logdebug("Updating physics attributes")

        self._time_step = physics.time_step
        self._max_update_rate = physics.max_update_rate
        self._gravity_x = physics.gravity.x
        self._gravity_y = physics.gravity.y
        self._gravity_z = physics.gravity.z
        self._auto_disable_bodies = physics.ode_config.auto_disable_bodies
        self._sor_pgs_precon_iters = physics.ode_config.sor_pgs_precon_iters
        self._sor_pgs_iters = physics.ode_config.sor_pgs_iters
        self._sor_pgs_w = physics.ode_config.sor_pgs_w
        self._sor_pgs_rms_error_tol = physics.ode_config.sor_pgs_rms_error_tol
        self._contact_surface_layer = physics.ode_config.contact_surface_layer
        self._contact_max_correcting_vel = physics.ode_config.contact_max_correcting_vel
        self._cfm = physics.ode_config.cfm
        self._erp = physics.ode_config.erp
        self._max_contacts = physics.ode_config.max_contacts

    def _set_physics(self, **kwargs):
        """Private method that update some physics parameter
        """
        if not kwargs:
            rospy.logwarn("An unnecessary request will be sent")

        self._get_physics()

        set_physics_request = self._build_set_physics_request(kwargs)

        self. _send_set_physics_request(set_physics_request)

    def _build_set_physics_request(self, kwargs):
        """Fill a SetPhysicsPropertiesRequest with properties in kwargs.
        Current values of properties are thr default values of the request.

        Args:
            kwargs (dict): physic properties (see SetPhysicsPropertiesRequest)

        Returns:
            SetPhysicsPropertiesRequest: request to be sent
        """
        set_physics_request = SetPhysicsPropertiesRequest()
        set_physics_request.time_step = kwargs.get(
            'time_step', self._time_step)
        set_physics_request.max_update_rate = kwargs.get(
            'max_update_rate', self._max_update_rate)
        gravity = Vector3()
        gravity.x = kwargs.get(
            'gravity_x', self._gravity_x)
        gravity.y = kwargs.get(
            'gravity_y', self._gravity_y)
        gravity.z = kwargs.get(
            'gravity_z', self._gravity_z)
        set_physics_request.gravity = gravity
        ode_config = ODEPhysics()
        ode_config.auto_disable_bodies = kwargs.get(
            'auto_disable_bodies', self._auto_disable_bodies)
        ode_config.sor_pgs_precon_iters = kwargs.get(
            'sor_pgs_precon_iters', self._sor_pgs_precon_iters)
        ode_config.sor_pgs_iters = kwargs.get(
            'sor_pgs_iters', self._sor_pgs_iters)
        ode_config.sor_pgs_w = kwargs.get(
            'sor_pgs_w', self._sor_pgs_w)
        ode_config.sor_pgs_rms_error_tol = kwargs.get(
            'sor_pgs_rms_error_tol', self._sor_pgs_rms_error_tol)
        ode_config.contact_surface_layer = kwargs.get(
            'contact_surface_layer', self._contact_surface_layer)
        ode_config.contact_max_correcting_vel = kwargs.get(
            'contact_max_correcting_vel', self._contact_max_correcting_vel)
        ode_config.cfm = kwargs.get(
            'cfm', self._cfm)
        ode_config.erp = kwargs.get(
            'erp', self._erp)
        ode_config.max_contacts = kwargs.get(
            'max_contacts', self._max_contacts)
        set_physics_request.ode_config = ode_config

        return set_physics_request

    def _send_set_physics_request(self, set_physics_request):
        """Send a set_physics_request. No need to pause the simulation.

        Args:
            set_physics_request (SetPhysicsPropertiesRequest): the request

        Raises:
            Exception: if the request is rejected
        """
        with self.pausing():
            rospy.logdebug("Sending the set_physics_request")
            result = self._set_physics_srv(set_physics_request)

        if not result.success:
            rospy.logerr(result.status_message)
            raise Exception(result.status_message)
        else:
            rospy.logdebug("Update physics succeed, %s" %
                           result.status_message)

    @contextmanager
    def pausing(self):
        """Pausing context
        """
        try:
            self.pause()
            yield
        finally:
            self.unpause()

if __name__ == '__main__':
    rospy.loginfo('i am here !!')