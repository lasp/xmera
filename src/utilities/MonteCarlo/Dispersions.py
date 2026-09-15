# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import abc
import collections
import random

import numpy as np
from xmera.utilities import RigidBodyKinematics as rbk
from xmera.utilities import orbitalMotion


class SingleVariableDispersion(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, var_name, bounds):
        self.var_name = var_name
        self.bounds = bounds
        self.magnitude = []

    @abc.abstractmethod
    def generate(self, sim):
        pass

    def get_dispersion_mag(self):
        return self.magnitude

    def check_bounds(self, value):
        if self.bounds is None:
            return value

        if value <= self.bounds[0]:
            value = self.bounds[0]
        if value >= self.bounds[1]:
            value = self.bounds[1]
        return value

    def get_name(self):
        return self.var_name

    def generate_string(self, sim):
        return str(self.generate(sim))

    def generate_mag_string(self):
        return str(self.get_dispersion_mag())


class UniformDispersion(SingleVariableDispersion):
    def __init__(self, var_name, bounds=None):
        SingleVariableDispersion.__init__(self, var_name, bounds)
        if self.bounds is None:
            self.bounds = ([-1.0, 1.0])  # defines a hard floor/ceiling

    def generate(self, sim):
        disp_value = random.uniform(self.bounds[0], self.bounds[1])

        mid = (self.bounds[1] + self.bounds[0])/2.
        scale = self.bounds[1] - mid
        self.magnitude.append(str(round((disp_value - mid)/scale*100,2)) + " %")
        return disp_value


class UniformDispersionSymmetricBounds(SingleVariableDispersion):
    def __init__(self, var_name, bounds=None):
        SingleVariableDispersion.__init__(self, var_name, bounds)
        if self.bounds is None:
             self.bounds = ([0.5, 1.0])  # defines a hard floor/ceiling

    def generate(self, sim):
        disp_value = random.uniform(self.bounds[0], self.bounds[1]) * random.choice([-1, 1])

        mid = 0.0
        scale = self.bounds[1] - mid
        self.magnitude.append(str(round((disp_value - mid)/scale*100,2)) + " %")
        return disp_value


class NormalDispersion(SingleVariableDispersion):
    def __init__(self, var_name, mean=0.0, std_deviation=0.5, bounds=None):
        SingleVariableDispersion.__init__(self, var_name, bounds)
        self.mean = mean
        self.std_deviation = std_deviation

    def generate(self, sim):
        disp_value = random.gauss(self.mean, self.std_deviation)
        if self.bounds is not None:
            disp_value = self.check_bounds(disp_value)
        if self.std_deviation !=0 :
            self.magnitude.append(str(round((disp_value - self.mean) / self.std_deviation, 2)) + " sigma")
        return disp_value


class VectorVariableDispersion(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, var_name, bounds):
        self.var_name = var_name
        self.bounds = bounds
        self.magnitude = []

    @abc.abstractmethod
    def generate(self, sim=None):
        pass

    def get_dispersion_mag(self):
        return self.magnitude

    def perturb_vector_by_angle(self, vector, angle):
        rnd_vec = np.random.random(3)
        if np.dot(rnd_vec, vector) > 0.95:
            rnd_vec[0] *= -1
        eigen_axis = np.cross(vector, rnd_vec)
        thruster_misalign_dcm = self.eig_axis_and_angle_to_dcm(eigen_axis, angle)
        return np.dot(thruster_misalign_dcm, vector)

    def perturb_cartesian_vector_uniform(self, vector):
        disp_values = np.zeros(3)
        for i in range(len(vector)):
            disp_values[i] = random.uniform(self.bounds[0], self.bounds[1])
            mid = (self.bounds[1] + self.bounds[0])
            scale = self.bounds[1] - mid
            self.magnitude.append(str(round((disp_values[i] - mid)/scale*100,2)) + " %")
        return disp_values

    def perturb_cartesian_vector_normal(self, vector):
        disp_values = np.zeros(3)
        for i in range(len(vector)):
            disp_values[i] = random.gauss(self.mean, self.std_deviation)
            if self.std_deviation != 0 :
                self.magnitude.append(str(round((disp_values[i] - self.mean)/self.std_deviation,2)) + r" $\sigma$")
        return disp_values

    def cart2_spherical(self, cart_vec):
        # Spherical Coordinate Set: [rho, theta, phi]
        x = cart_vec[0]
        y = cart_vec[1]
        z = cart_vec[2]

        rho = np.linalg.norm(cart_vec)
        phi = np.arctan2(y, x)[0]
        theta = np.arccos(z)[0]

        return [rho, phi, theta]

    def spherical2_cart(self, spher_vec):
        rho = spher_vec[0]
        phi = spher_vec[1]
        theta = spher_vec[2]

        x = rho * np.sin(theta) * np.cos(phi)
        y = rho * np.sin(theta) * np.sin(phi)
        z = rho * np.cos(theta)

        return [x,y,z]

    @staticmethod
    def eig_axis_and_angle_to_dcm(axis, angle):
        axis = axis / np.linalg.norm(axis)
        sigma = 1 - np.cos(angle)
        dcm = np.zeros((3, 3))
        dcm[0, 0] = axis[0] ** 2 * sigma + np.cos(angle)
        dcm[0, 1] = axis[0] * axis[1] * sigma + axis[2] * np.sin(angle)
        dcm[0, 2] = axis[0] * axis[2] * sigma - axis[1] * np.sin(angle)
        dcm[1, 0] = axis[1] * axis[0] * sigma - axis[2] * np.sin(angle)
        dcm[1, 1] = axis[1] ** 2 * sigma + np.cos(angle)
        dcm[1, 2] = axis[1] * axis[2] * sigma + axis[0] * np.sin(angle)
        dcm[2, 0] = axis[2] * axis[0] * sigma + axis[1] * np.sin(angle)
        dcm[2, 1] = axis[2] * axis[1] * sigma - axis[0] * np.sin(angle)
        dcm[2, 2] = axis[2] ** 2 * sigma + np.cos(angle)
        return dcm

    # @TODO This should be a @classmethod.
    @staticmethod
    def check_bounds(value, bounds):
        if value < bounds[0]:
            value = bounds[0]
        if value > bounds[1]:
            value = bounds[1]
        return value

    def generate_string(self, sim):
        # TODO does this actually behave differently then str(next_value)?
        next_value = self.generate(sim)
        val = '['
        for i in range(3):
            val += str(next_value[i]) + ','
        val = val[0:-1] + ']'
        return val

    def generate_mag_string(self):
        next_value = self.get_dispersion_mag()
        val = '['
        for i in range(len(self.magnitude)):
            val += str(next_value[i]) + ','
        val = val[0:-1] + ']'
        return val

    def get_name(self):
        return self.var_name


class UniformVectorDispersion(VectorVariableDispersion):
    def __init__(self, var_name, bounds=None):
        VectorVariableDispersion.__init__(self, var_name, bounds)
        if self.bounds is None:
            self.bounds = ([-1.0, 1.0])  # defines a hard floor/ceiling

    def generate(self, sim):
        vector = eval('sim.' + self.var_name)
        disp_value = self.perturb_cartesian_vector_uniform(vector)
        return disp_value


class NormalVectorDispersion(VectorVariableDispersion):
    def __init__(self, var_name, mean=0.0, std_deviation=0.5, bounds=None):
        VectorVariableDispersion.__init__(self, var_name, bounds)
        if self.bounds is None:
            self.bounds = ([-1.0, 1.0])  # defines a hard floor/ceiling

    def generate(self, sim):
        vector = eval('sim.' + self.var_name)
        disp_value = self.perturb_cartesian_vector_normal(vector, self.mean, self.std_deviation)
        return disp_value


class UniformVectorAngleDispersion(VectorVariableDispersion):
    def __init__(self, var_name, phi_bounds_off_nom=None, theta_bounds_off_nom=None):
        super(UniformVectorAngleDispersion, self).__init__(var_name, None)
        # @TODO these bounds are not currently being applied to the generated values

        self.phi_bounds_off_nom = phi_bounds_off_nom
        self.theta_bounds_off_nom = theta_bounds_off_nom

        if phi_bounds_off_nom is None:
            self.phi_bounds_off_nom = [-np.pi / 2, np.pi / 2]
        if theta_bounds_off_nom is None:
            self.theta_bounds_off_nom = [-np.pi, np.pi]

        self.magnitude = []

    def generate(self, sim=None):
        # Note this dispersion is applied off of the nominal
        vector_cart = eval('sim.' + self.var_name)
        vector_cart = vector_cart/np.linalg.norm(vector_cart)
        vector_sphere = self.cart2_spherical(vector_cart)

        mean_phi = vector_sphere[1] # Nominal phi
        mean_theta = vector_sphere[2] #Nominal theta

        self.phi_bounds = [mean_phi + self.phi_bounds_off_nom[0], mean_phi + self.phi_bounds_off_nom[1]]
        self.theta_bounds = [mean_theta + self.theta_bounds_off_nom[0], mean_theta + self.theta_bounds_off_nom[1]]

        phi_rnd = np.random.uniform(mean_phi + self.phi_bounds[0], mean_phi + self.phi_bounds[1])
        theta_rnd = np.random.uniform(mean_theta + self.theta_bounds[0], mean_theta + self.theta_bounds[1])

        phi_rnd = self.check_bounds(phi_rnd, self.phi_bounds)
        theta_rnd = self.check_bounds(theta_rnd, self.theta_bounds)

        new_vec = self.spherical2_cart([1.0, phi_rnd, theta_rnd])
        disp_vec = new_vec/np.linalg.norm(new_vec) # Shouldn't technically need the normalization but doing it for completeness

        mid_phi = (self.phi_bounds[1] + self.phi_bounds[0]) / 2.
        scale_phi = self.phi_bounds[1] - mid_phi
        mid_theta = (self.theta_bounds[1] + self.theta_bounds[0]) / 2.
        scale_theta = self.theta_bounds[1] - mid_theta
        self.magnitude.append(str(round((phi_rnd - mid_phi)/scale_phi*100,2)) + " %")
        self.magnitude.append(str(round((theta_rnd - mid_theta)/scale_theta*100,2)) + " %")

        return disp_vec


class NormalVectorAngleDispersion(VectorVariableDispersion):
    def __init__(self, var_name, theta_std =np.pi / 3.0, phi_std=np.pi / 3.0, theta_bounds_off_nom=None, phi_bounds_off_nom=None):
        super(NormalVectorAngleDispersion, self).__init__(var_name, None)
        # @TODO these bounds are not currently being applied to the generated values

        self.theta_std = theta_std
        self.phi_std = phi_std

        self.phi_bounds_off_nom = phi_bounds_off_nom
        self.theta_bounds_off_nom = theta_bounds_off_nom

        if phi_bounds_off_nom is None:
            self.phi_bounds_off_nom = [-np.pi / 2, np.pi / 2]
        if theta_bounds_off_nom is None:
            self.theta_bounds_off_nom = [-np.pi, np.pi]

        self.magnitude = []

    def generate(self, sim=None):
        vector_cart = eval('sim.' + self.var_name)
        vector_cart = vector_cart/np.linalg.norm(vector_cart)
        vector_sphere = self.cart2_spherical(vector_cart)

        mean_phi = vector_sphere[1] # Nominal phi
        mean_theta = vector_sphere[2] # Nominal theta

        phi_rnd = np.random.normal(mean_phi, self.phi_std)
        theta_rnd = np.random.normal(mean_theta, self.theta_std)

        self.phiBounds = [mean_phi + self.phi_bounds_off_nom[0], mean_phi + self.phi_bounds_off_nom[1]]
        self.thetaBounds = [mean_theta + self.theta_bounds_off_nom[0], mean_theta + self.theta_bounds_off_nom[1]]

        phi_rnd = self.check_bounds(phi_rnd, self.phiBounds)
        theta_rnd = self.check_bounds(theta_rnd, self.thetaBounds)

        new_vec = self.spherical2_cart([1.0, phi_rnd, theta_rnd])
        disp_vec = new_vec/np.linalg.norm(new_vec) # Shouldn't technically need the normalization but doing it for completeness

        self.magnitude.append(str(round((phi_rnd - mean_phi) / self.phi_std, 2)) + r" $\sigma$")
        self.magnitude.append(str(round((theta_rnd - mean_theta) / self.theta_std, 2)) + r" $\sigma$")

        return disp_vec


class UniformVectorSingleAngleDispersion(VectorVariableDispersion):
    def __init__(self, var_name, bounds=None):
        super(UniformVectorSingleAngleDispersion, self).__init__(var_name, None)

        self.bounds = bounds

        if bounds is None:
            self.bounds = [-np.pi, np.pi]

        self.magnitude = []

    def generate(self, sim=None):
        dir_vec = eval('sim.' + self.var_name)
        angle = np.random.uniform(self.bounds[0], self.bounds[1])
        angle = self.check_bounds(angle, self.bounds)
        dir_vec = np.array(dir_vec).reshape(3).tolist()
        disp_vec = self.perturb_vector_by_angle(dir_vec, angle)
        angle_disp = np.arccos(np.dot(dir_vec, disp_vec)/np.linalg.norm(dir_vec)/np.linalg.norm(disp_vec))
        mid_angle = (self.bounds[1] + self.bounds[0])/2.
        scale_angle = self.bounds[1] - mid_angle
        self.magnitude.append(str(round((angle_disp - mid_angle)/scale_angle*100, 2)) + " %")

        return disp_vec


class NormalVectorSingleAngleDispersion(VectorVariableDispersion):
    def __init__(self, var_name, phi_std=np.pi / 36.0, bounds=None):
        super(NormalVectorSingleAngleDispersion, self).__init__(var_name, None)

        self.phi_std = phi_std
        self.bounds = bounds

        if bounds is None:
            self.bounds = [-np.pi, np.pi]

        self.magnitude = []

    def generate(self, sim=None):
        dir_vec = eval('sim.' + self.var_name)
        angle = np.random.normal(0, self.phi_std)
        angle = self.check_bounds(angle, self.bounds)
        dir_vec = np.array(dir_vec).reshape(3).tolist()
        disp_vec = self.perturb_vector_by_angle(dir_vec, angle)
        angle_disp = np.arccos(np.dot(dir_vec, disp_vec)/np.linalg.norm(dir_vec)/np.linalg.norm(disp_vec))
        self.magnitude.append(str(round(angle_disp / self.phi_std, 2)) + " sigma")

        return disp_vec


class UniformEulerAngleMRPDispersion(VectorVariableDispersion):
    def __init__(self, var_name, bounds=None):
        """
        Args:
            var_name (str): A string representation of the variable to be dispersed
                e.g. 'VehDynObject.AttitudeInit'.
            bounds (Array[float, float]): defines lower and upper cut offs for generated dispersion values radians.
        """
        super(UniformEulerAngleMRPDispersion, self).__init__(var_name, bounds)
        if self.bounds is None:
            self.bounds = ([0, 2 * np.pi])
        self.magnitude = []

    def generate(self, sim=None):
        rnd_angles = np.zeros((3, 1))
        for i in range(3):
            rnd_angles[i] = (self.bounds[1] - self.bounds[0]) * np.random.random() + self.bounds[0]
        disp_mrp = rbk.euler3232MRP(rnd_angles)
        disp_mrp = disp_mrp.reshape(3)
        for i in range(3):
            self.magnitude.append(str(round((disp_mrp[i] - np.pi)/np.pi*100,2))+ " %")
        return disp_mrp


class NormalThrusterUnitDirectionVectorDispersion(VectorVariableDispersion):
    def __init__(self, var_name, thruster_index=0, phi_std=0.1745, bounds=None):
        """
        Args:
            var_name (str): A string representation of the variable to be dispersed
                e.g. 'ACSThrusterDynObject.ThrusterData[0].thrusterDirectionDisp'.
            thruster_index (int): The index of the thruster to be used in array references.
            phi_std (float): The 1 sigma standard deviation of the dispersion angle in radians.
            bounds (Array[float, float]): defines lower and upper cut offs for generated dispersion values.
        """
        super(NormalThrusterUnitDirectionVectorDispersion, self).__init__(var_name, bounds)
        self.var_name_components = self.var_name.split(".")
        self.phi_std = phi_std  # (rad) angular standard deviation
        # Limit dispersion to a hemisphere around the vector being dispersed
        # if self.bounds is None:
        #     self.bounds = ([-np.pi/2, np.pi/2])
        self.thruster_index = thruster_index
        self.magnitude = []

    def get_name(self):
        return '.'.join(self.var_name_components[0:-1]) + '.thrDir_B'

    def generate_string(self, sim):
        # TODO does this actually behave differently then str(next_value)?
        next_value = self.generate(sim)

        val = '['
        for i in range(3):
            val += str(next_value[i])
            if i < 2:
                val += ', '
        val += ']'

        return val

    def generate_mag_string(self):
        next_value = self.get_dispersion_mag()

        val = '['
        for i in range(len(self.magnitude)):
            val += str(next_value[i])
            val += ', '
        val += ']'

        return val

    def generate(self, sim=None):
        if sim is None:
            print(("No simulation object parameter set in '" + self.generate.__name__
                   + "()' dispersions will not be set for variable " + self.var_name))
            return
        else:
            separator = '.'
            thruster_object = getattr(sim, self.var_name_components[0])
            total_var = separator.join(self.var_name_components[0:-1])
            dir_vec = eval('sim.' + total_var + '.thrDir_B')
            angle = np.random.normal(0, self.phi_std, 1)
            dir_vec = np.array(dir_vec).reshape(3).tolist()
            disp_vec = self.perturb_vector_by_angle(dir_vec, angle)
            angle_disp = np.arccos(np.dot(dir_vec, disp_vec)/np.linalg.norm(dir_vec)/np.linalg.norm(disp_vec))
            self.magnitude.append(str(round(angle_disp / self.phi_std, 2)) + " sigma")
        return disp_vec


class UniformVectorCartDispersion(VectorVariableDispersion):
    def __init__(self, var_name, bounds=None):
        super(UniformVectorCartDispersion, self).__init__(var_name, bounds)
        if self.bounds is None:
            self.bounds = ([-1.0, 1.0])
        self.magnitude = []

    def generate(self, sim=None):
        disp_vec = []
        for i in range(3):
            rnd = random.uniform(self.bounds[0], self.bounds[1])
            rnd = self.check_bounds(rnd, self.bounds)
            for i in range(3):
                mid = (self.bounds[1] + self.bounds[0])/2.
                scale = self.bounds[1] - mid
                self.magnitude.append(str(round((rnd - mid) / scale * 100,2))+ " %")
            disp_vec.append(rnd)
        return disp_vec


class NormalVectorCartDispersion(VectorVariableDispersion):
    def __init__(self, var_name, mean=0.0, std_deviation=0.0, bounds=None):
        super(NormalVectorCartDispersion, self).__init__(var_name, bounds)
        self.mean = mean
        self.std_deviation = std_deviation
        self.magnitude = []

    def generate(self, sim=None):
        disp_vec = []
        for i in range(3):
            if isinstance(self.std_deviation, collections.abc.Sequence):
                rnd = random.gauss(self.mean[i], self.std_deviation[i])
                if self.std_deviation[i] != 0:
                    self.magnitude.append(str(round((rnd - self.mean[i])/self.std_deviation[i],2)) + " sigma")
            else:
                rnd = random.gauss(self.mean, self.std_deviation)
                if self.std_deviation != 0:
                    self.magnitude.append(str(round((rnd - self.mean) / self.std_deviation,2)) + " sigma")
            if self.bounds is not None:
                rnd = self.check_bounds(rnd, self.bounds)
            disp_vec.append(rnd)

        return disp_vec


class InertiaTensorDispersion:
    def __init__(self, var_name, std_diag=None, bounds_diag=None, std_angle=None):
        """
        Args:
            var_name (str): A string representation of the variable to be dispersed
                e.g. 'LocalConfigData.I'.
            std_deviation (float): The 1 sigma standard deviation of the diagonal element dispersions in kg*m^2.
            bounds (Array[float, float]): defines lower and upper cut offs for generated dispersion values kg*m^2.
        """
        self.var_name = var_name
        self.var_name_components = self.var_name.split(".")
        self.std_diag = std_diag
        self.std_angle = std_angle
        self.bounds = bounds_diag
        self.magnitude = []
        if self.std_diag is None:
            self.std_diag = 1.0
        if self.bounds is None:
            self.bounds = ([-1.0, 1.0])
        if self.std_angle is None:
            self.std_angle = 0.0

    def generate(self, sim=None):
        if sim is None:
            print(("No simulation object parameter set in '" + self.generate.__name__
                   + "()' dispersions will not be set for variable " + self.var_name))
            return
        else:
            I = np.array(eval('sim.' + self.var_name)).reshape(3, 3)

            # generate random values for the diagonals
            temp = []
            for i in range(3):
                rnd = random.gauss(0, self.std_diag)
                rnd = self.check_bounds(rnd)
                temp.append(rnd)
                if self.std_diag != 0:
                    self.magnitude.append(str(round(rnd / self.std_diag, 2)) + " sigma")
            disp_identity_matrix = np.identity(3) * temp
            # generate random values for the similarity transform to produce off-diagonal terms
            angles = np.random.normal(0, self.std_angle, 3)
            for i in range(3):
                if self.std_angle != 0:
                    self.magnitude.append(str(round(angles[i] / self.std_angle, 2)) + " sigma")
            disp321_matrix = rbk.euler3212C(angles)

            # disperse the diagonal elements
            disp_I = I + disp_identity_matrix
            # disperse the off diagonals with a slight similarity transform of the inertia tensor
            disp_I = np.dot(np.dot(disp321_matrix, disp_I), disp321_matrix.T)

        return disp_I

    def get_dispersion_mag(self):
        return self.magnitude

    def check_bounds(self, value):
        if value < self.bounds[0]:
            value = self.bounds[0]
        if value > self.bounds[1]:
            value = self.bounds[1]
        return value

    def generate_string(self, sim):
        next_value = self.generate(sim)
        # TODO does this actually behave differently then str(next_value)?
        val = '['
        for i in range(3):
            val += '[' + str(next_value[i][0]) + ', ' \
                + str(next_value[i][1]) + ', ' \
                + str(next_value[i][2]) + ']'
            if i != 2:
                val += ','
        val = val[0:] + ']'
        return val

    def generate_mag_string(self):
        next_value = self.get_dispersion_mag()
        val = '['
        for i in range(len(self.magnitude)):
            val += str(next_value[i]) + ','
        val = val[0:-1] + ']'
        return val

    def get_name(self):
        return self.var_name


class OrbitalElementDispersion:
    def __init__(self, var_name1, var_name2, disp_dict):
        """
        A function that disperses position and velocity of the spacecraft using orbital elements as a dispersion metric.
        Args:
            var_name1 (str): A string representation of the position variable to be dispersed
            var_name2 (str): A string representation of the velocity variable to be dispersed
            disp_dict (dict): A dictionnary containing the dispersions for each of the orbital elements. The values are lists
            with first element 'normal' or 'uniform' followed by mean, std or lower bound, upper bound respectively. If no dispersion
            is added for a specific orbital elemenet, None should be the values for the corresponding key
        """
        self.number_of_sub_disps = 2
        self.var_name1 = var_name1
        self.var_name1_components = self.var_name1.split(".")
        self.var_name2 = var_name2
        self.var_name2_components = self.var_name2.split(".")
        self.oe_dict = disp_dict


    def generate(self, sim=None):
        elems = orbitalMotion.ClassicElements
        for key in self.oe_dict.keys():
            if self.oe_dict[key] is not None and key != "mu":
                exec("elems." + key + " = np.random." + self.oe_dict[key][0] + "(" + str(self.oe_dict[key][1]) + ', ' + str(self.oe_dict[key][2]) + ")")
            else:
                if key != "mu":
                    exec("elems." + key + " = 0.")
        if elems.e < 0:
            elems.e = 0
        r, v =orbitalMotion.elem2rv_parab(self.oe_dict["mu"], elems)

        self.disp_r = r
        self.disp_v = v


    def generate_string(self, index, sim=None):
        if index == 1:
            next_value = self.disp_r
        if index == 2:
            next_value = self.disp_v
        val = '['
        for i in range(3):
            val += str(next_value[i]) + ','
        val = val[0:-1] + ']'
        return val

    def get_name(self, index):
        if index == 1:
            return self.var_name1
        if index == 2:
            return self.var_name2

class MRPDispersionPerAxis(VectorVariableDispersion):
    def __init__(self, var_name, bounds=None):
        """
        A function that disperses MRPs with specfic bounds per axis.
        Args:
            var_name (str): A string representation of the variable to be dispersed
                e.g. 'VehDynObject.AttitudeInit'.
            bounds (list(Array[float, float],Array[float, float],Array[float, float])): defines lower and upper cut offs for generated dispersion values radians.
        """
        super(MRPDispersionPerAxis, self).__init__(var_name, bounds)
        if self.bounds is None:
            self.bounds = [[0, 2 * np.pi], [0, 2 * np.pi], [0, 2 * np.pi]]

    def generate(self, sim=None):
        rnd_angles = np.zeros((3, 1))
        for i in range(3):
            rnd_angles[i] = (self.bounds[i][1] - self.bounds[i][0]) * np.random.random() + self.bounds[i][0]
        disp_mrp = rnd_angles.reshape(3)
        return disp_mrp


class SymmetricSolarArrayDispersion():
    def __init__(self, angle1_dyn_str, angle2_dyn_str, angle1_controller_str, angle2_controller_str, angle1_profiler_str,
                 angle2_profiler_str, bounds=None):
        self.angle1_dyn_str = angle1_dyn_str
        self.angle2_dyn_str = angle2_dyn_str
        self.angle1_controller_str = angle1_controller_str
        self.angle2_controller_str = angle2_controller_str
        self.angle1_profiler_str = angle1_profiler_str
        self.angle2_profiler_str = angle2_profiler_str
        self.bounds = bounds
        self.number_of_sub_disps = 6

    def generate(self, sim=None):
        disp_value = random.uniform(self.bounds[0], self.bounds[1])
        self.angle1_dyn_val = disp_value
        self.angle2_dyn_val = -disp_value
        self.angle1_controller_val = disp_value
        self.angle2_controller_val = -disp_value
        self.angle1_profiler_val = disp_value
        self.angle2_profiler_val = -disp_value

    def generate_string(self, index, sim=None):
        if index == 1:
            next_value = self.angle1_dyn_val
        if index == 2:
            next_value = self.angle2_dyn_val
        if index == 3:
            next_value = self.angle1_controller_val
        if index == 4:
            next_value = self.angle2_controller_val
        if index == 5:
            next_value = self.angle1_profiler_val
        if index == 6:
            next_value = self.angle2_profiler_val
        val = str(next_value)
        return val

    def get_name(self, index):
        if index == 1:
            return self.angle1_dyn_str
        if index == 2:
            return self.angle2_dyn_str
        if index == 3:
            return self.angle1_controller_str
        if index == 4:
            return self.angle2_controller_str
        if index == 5:
            return self.angle1_profiler_str
        if index == 6:
            return self.angle2_profiler_str


class SymmetricSolarArrayWithReferenceDispersion():
    def __init__(self, angle1_dyn_str, angle2_dyn_str, angle1_controller_str, angle2_controller_str, angle1_profiler_str,
                 angle2_profiler_str, ref_angle1_str, ref_angle2_str, bounds=None):
        self.angle1_dyn_str = angle1_dyn_str
        self.angle2_dyn_str = angle2_dyn_str
        self.angle1_controller_str = angle1_controller_str
        self.angle2_controller_str = angle2_controller_str
        self.angle1_profiler_str = angle1_profiler_str
        self.angle2_profiler_str = angle2_profiler_str
        self.ref_angle1_str = ref_angle1_str
        self.ref_angle2_str = ref_angle2_str
        self.bounds = bounds
        self.number_of_sub_disps = 8

    def generate(self, sim=None):
        disp_value = random.uniform(self.bounds[0], self.bounds[1])

        self.angle1_dyn_val = disp_value
        self.angle2_dyn_val = -disp_value
        self.angle1_controller_val = disp_value
        self.angle2_controller_val = -disp_value
        self.angle1_profiler_val = disp_value
        self.angle2_profiler_val = -disp_value
        self.ref_angle1_val = disp_value
        self.ref_angle2_val = -disp_value

    def generate_string(self, index, sim=None):
        if index == 1:
            next_value = self.angle1_dyn_val
        if index == 2:
            next_value = self.angle2_dyn_val
        if index == 3:
            next_value = self.angle1_controller_val
        if index == 4:
            next_value = self.angle2_controller_val
        if index == 5:
            next_value = self.angle1_profiler_val
        if index == 6:
            next_value = self.angle2_profiler_val
        if index == 7:
            next_value = self.ref_angle1_val
        if index == 8:
            next_value = self.ref_angle2_val
        val = str(next_value)
        return val

    def get_name(self, index):
        if index == 1:
            return self.angle1_dyn_str
        if index == 2:
            return self.angle2_dyn_str
        if index == 3:
            return self.angle1_controller_str
        if index == 4:
            return self.angle2_controller_str
        if index == 5:
            return self.angle1_profiler_str
        if index == 6:
            return self.angle2_profiler_str
        if index == 7:
            return self.ref_angle1_str
        if index == 8:
            return self.ref_angle2_str
