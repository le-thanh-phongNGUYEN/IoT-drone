import logging
import time
import math
from threading import Thread

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

import numpy as np
from filterpy.kalman import KalmanFilter
import scipy.io


logging.basicConfig(level=logging.ERROR)


class AltHoldExample:
    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """
        self._f1 = open("data_position.csv", "w")
        self._f2 = open("data_stabilizer.csv", "w")
        self._f3 = open("data_parameters.csv", "w")

        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)
        self.is_connected = True
        # Variable used to keep main loop occupied until disconnect
        print('Connecting to %s' % link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        self._lg_stab = LogConfig(name='Position', period_in_ms=10)
        self._lg_stab.add_variable('kalman.stateX', 'float')
        self._lg_stab.add_variable('kalman.stateY', 'float')
        self._lg_stab.add_variable('kalman.stateZ', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')

        self._lg_stab1 = LogConfig(name='Stabilizer', period_in_ms=10)
        self._lg_stab1.add_variable('stabilizer.pitch', 'float')
        self._lg_stab1.add_variable('stabilizer.roll', 'float')
        self._lg_stab1.add_variable('stabilizer.thrust', 'float')

        try:
            self._cf.log.add_config(self._lg_stab)
            self._cf.log.add_config(self._lg_stab1)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            self._lg_stab1.data_received_cb.add_callback(self._stab_log_data1)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # self._lg_stab1.error_cb.add_callback(self._stab_log_error1)
            # Start the logging
            self._lg_stab.start()
            self._lg_stab1.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._hover_test).start()

    def updateX(self, value, targetValue, dt):
        kp = 30.0
        kd = 0.0
        ki = 3
        minOutput = -3.0
        maxOutput = 3.0
        integratorMin = -0.3
        integratorMax = 0.3
        
        global boolSaveParametersX
        if (boolSaveParametersX == True):
            self._f3.write("{},".format(maxOutput))
            self._f3.write("{},".format(kp))
            self._f3.write("{},".format(ki))
            self._f3.write("{},".format(kd))
            boolSaveParametersX = False

        global integral_X, previousError_X
        error = targetValue - value
        integral_X = integral_X + error * dt
        integral_X = max(min(integral_X, integratorMax), integratorMin)

        p = kp * error
        d = kd * (error - previousError_X) / dt
        i = ki * integral_X

        output = p + d + i
        previousError_X = error

        return max(min(output, maxOutput), minOutput)

    def updateY(self, value, targetValue, dt):
        kp = -30.0
        kd = 0.0
        ki = -3.0
        minOutput = -3.0
        maxOutput = 3.0
        integratorMin = -0.3
        integratorMax = 0.3
        
        global boolSaveParametersY
        if (boolSaveParametersY == True):
            self._f3.write("{},".format(maxOutput))
            self._f3.write("{},".format(kp))
            self._f3.write("{},".format(ki))
            self._f3.write("{},".format(kd))
            boolSaveParametersY = False

        global integral_Y, previousError_Y
        error = targetValue - value
        integral_Y = integral_Y + error * dt
        integral_Y = max(min(integral_Y, integratorMax), integratorMin)

        p = kp * error
        d = kd * (error - previousError_Y) / dt
        i = ki * integral_Y

        output = p + d + i
        previousError_Y = error

        return max(min(output, maxOutput), minOutput)

    def updateZ(self, value, targetValue, dt):
        # KiN = Ki
        # KpN = Kd
        # KpN = Kp + Ki
        # kp = 9000.0
        # kd = 11000.0
        # ki = 3500.0
        kp = 12000.0
        kd = 0.0
        ki = 3500.0
        minOutput = -20000.0
        maxOutput = 20000.0
        integratorMin = -1000.0
        integratorMax = 1000.0
        
        global boolSaveParametersZ
        if (boolSaveParametersZ == True):
            self._f3.write("{},".format(maxOutput))
            self._f3.write("{},".format(minOutput))
            self._f3.write("{},".format(kp))
            self._f3.write("{},".format(ki))
            self._f3.write("{},".format(kd))
            boolSaveParametersZ=False

        global integral_Z, previousError_Z
        error = targetValue - value
        integral_Z = integral_Z + error * dt
        integral_Z = max(min(integral_Z, integratorMax), integratorMin)

        p = kp * error
        d = kd * (error - previousError_Z) / dt
        i = ki * integral_Z

        output = p + d + i
        previousError_Z = error

        return max(min(output, maxOutput), minOutput)

    def updateYaw(self, value, targetValue, dt):
        kp = -3.0
        kd = 0.0
        ki = 0.0
        minOutput = -200.0
        maxOutput = 200.0
        integratorMin = 0.0
        integratorMax = 0.0
        
        global boolSaveParametersYaw
        if (boolSaveParametersYaw == True):
            self._f3.write("{},".format(kp))
            self._f3.write("{},".format(ki))
            self._f3.write("{},".format(kd))
            boolSaveParametersYaw = False

        global integral_Yaw, previousError_Yaw
        error = targetValue - value
        integral_Yaw = integral_Yaw + error * dt
        integral_Yaw = max(min(integral_Yaw, integratorMax), integratorMin)

        p = kp * error
        d = kd * (error - previousError_Yaw) / dt
        i = ki * integral_Yaw

        output = p + d + i
        previousError_Yaw = error

        return max(min(output, maxOutput), minOutput)

    def _stab_log_data(self, timestamp, data, logconf):
        global z_ref, k
        zref = z_ref[1,1]
        print('k = %f zref = %f' % (k, zref))
        k = k + 1

    def _stab_log_data1(self, timestamp, data, logconf):
        self._f2.write("\n {},".format(timestamp))
        self._f2.write("{},".format(data["stabilizer.pitch"]))
        self._f2.write("{},".format(data["stabilizer.roll"]))
        self._f2.write("{}".format(data["stabilizer.thrust"]))

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_error1(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

    def _hover_test(self):
        print("sending initial thrust of 0")

        self._cf.commander.send_setpoint(0, 0, 0, 0);
        # send_hover_setpoint
        time.sleep(0.5);

        it = 0

        while it < 470:
            time.sleep(0.1)
            it += 1

        print("Close connection")
        self._f1.close()
        self._f2.close()
        self._f3.close()
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(1);
        self._cf.close_link()


if __name__ == '__main__':
    x_pos = 0;
    y_pos = 0;
    z_pos = 0;
    k = 0;

    xref_pos = 0;
    yref_pos = 0;
    zref_pos = 0;

    # PID X
    integral_X = 0;
    previousError_X = 0;

    # PID Y
    integral_Y = 0;
    previousError_Y = 0;

    # PID Z
    integral_Z = 0;
    previousError_Z = 0;

    # PID Yaw
    integral_Yaw = 0;
    previousError_Yaw = 0;

    # Filter Kalman
    dt = 0.01
    f = KalmanFilter(dim_x=6, dim_z=3)
    f.x = np.array([[2.],
                    [2.],
                    [2.],
                    [0.],
                    [0.],
                    [0.]])  # initial state (location and velocity)

    f.F = np.array([[1., 0., 0., dt, 0., 0.],
                    [0., 1., 0., 0., dt, 0.],
                    [0., 0., 1., 0., 0., dt],
                    [0., 0., 0., 1., 0., 0.],
                    [0., 0., 0., 0., 1., 0.],
                    [0., 0., 0., 0., 0., 1.]])  # state transition matrix

    f.H = np.array([[1., 0., 0., 0., 0., 0.],
                    [0., 1., 0., 0., 0., 0.],
                    [0., 0., 1., 0., 0., 0.]])  # Measurement function

    G = np.array([[dt / 2, 0., 0.],
                  [0., dt / 2, 0.],
                  [0., 0., dt / 2],
                  [1., 0., 0.],
                  [0., 1., 0.],
                  [0., 0., 1.]])

    a11 = 3 * (10 ** -5)
    a12 = 8 * (10 ** -8)
    Q = np.array([[a11, 0., 0.],
                  [0., a11, 0.],
                  [0., 0., a12]])
    A = G.dot(Q)
    f.Q = A.dot(G.T)

    a21 = 5 * (10 ** -5)
    a22 = 5 * (10 ** -9)
    f.R = np.array([[a21, 0., 0.],
                    [0., a21, 0.],
                    [0., 0., a22]])  # state uncertainty

    zhover = scipy.io.loadmat('zhover.mat')
    z_ref = zhover['zhover']
    zref = z_ref[1, 300]
    print('k = %f zref = %f row = %f' % (k, zref, z_ref.shape[0]))
    print('String %s' % (z_ref))

    boolSaveParameters = True
    boolSaveParametersX= True
    boolSaveParametersY = True
    boolSaveParametersZ = True
    boolSaveParametersYaw = True
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    le = AltHoldExample("radio://0/110/2M")
