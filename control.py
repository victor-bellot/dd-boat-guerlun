import time
import math
from tools import *
from imu9_driver_v3 import Imu9IO
from tc74_driver_v2 import TempTC74IO
from arduino_driver_v2 import ArduinoIO
from encoders_driver_v2 import EncoderIO


class Control:
    def __init__(self, mission_name, dt=0.5):
        self.mission_name = mission_name
        self.log = open("log_files/log_%s.txt" % mission_name, 'a')
        self.traj = open("traj_files/traj_%s.txt" % mission_name, 'a')

        self.ard = ArduinoIO()
        self.enc = EncoderIO()
        self.imu = Imu9IO()
        self.tpr = TempTC74IO()
        self.gpsm = GpsManager()

        self.dt = dt
        # set delay between old and new measures : HERE=dt
        self.enc.set_older_value_delay_v2(int(dt * 10))
        self.tpr.set_config(0x0, 0x0)
        self.tpr.set_mode(standby=True, side="both")

        self.cst = {'left': {'kp': 0.01, 'ki': 0.01},
                    'right': {'kp': 0.01, 'ki': 0.01},
                    'phi': {'kp': (3/4) / np.pi, 'ki': 1e-2 / np.pi},
                    'line': {'kd': 150, 'kn': 1},
                    'left_': {'kd': 2, 'kp': 0.5, 'ki': 0.1},
                    'right_': {'kd': 2, 'kp': 0.5, 'ki': 0.1},
                    }

        self.step_max = 50
        self.u_max = 100
        self.rpm_max = 4000

        self.el_left, self.el_right = 0, 0
        self.ei_left, self.ei_right, self.ei_psi = 0, 0, 0
        self.cmd_left, self.cmd_right = 50, 50

    def reset(self, cmd_left_init=50, cmd_right_init=50):
        self.el_left, self.el_right = 0, 0
        self.ei_left, self.ei_right, self.ei_psi = 0, 0, 0
        self.cmd_left, self.cmd_right = cmd_left_init, cmd_right_init

    def close(self):
        self.log.close()
        self.traj.close()

    def change_timing(self, dt):
        self.dt = dt
        self.enc.set_older_value_delay_v2(int(dt * 10))

    def get_current_cap(self):
        return self.imu.orientation()

    def get_current_cap_degree(self):
        return self.get_current_cap() * (180 / np.pi)

    def line_to_phi_bar(self, line):
        """prend en argument une ligne (du type line définit dans le fichier tools.py);
        renvoie le cap à suivre pour suivre la ligne entrée en argument (line) (en s'appuyant sur la position gps des extremités de la ligne et celle mesurée par le bateau);
        écrit également les coordonnées du bateau dans le fichier traj, ces coordonnées pourront être récupérées à postériori pour tracer la trajectoire du bateau"""
        coord_boat = self.gpsm.coord

        if self.gpsm.updated:
            pos_boat = coord_to_pos(coord_boat)
            x, y = pos_boat.flatten()
            self.traj.write("%f;%f\n" % (x, y))

            kd, kn = self.cst['line']['kd'], self.cst['line']['kn']
            force = get_force(line, pos_boat, kd, kn)

            fx, fy = force.flatten()
            return np.arctan2(-fx, fy)

    def get_rpm(self):
        """renvoie la vitesse de rotation des moteurs mesurée par les encodeurs (en rotation par minute)"""
        # 1 : new ; 0 : old
        st1, st0 = self.enc.get_last_and_older_values_v2()
        data_encoders0 = np.array(st0.split(",")).astype(np.float)
        data_encoders1 = np.array(st1.split(",")).astype(np.float)

        odo_left0 = data_encoders0[4]
        odo_right0 = data_encoders0[3]

        odo_left1 = data_encoders1[4]
        odo_right1 = data_encoders1[3]

        rpm_left = (60. / 8.) * delta_odo(odo_left1, odo_left0) / self.dt
        rpm_right = (60. / 8.) * delta_odo(odo_right1, odo_right0) / self.dt

        return rpm_left, rpm_right

    def regulation_rpm(self, rpm_left_bar, rpm_right_bar):
        """prend en argument une consigne de vitesse de rotation pour chaque moteur (rpm_left_bar pour le moteur gauche et rpm_right_bar pour le moteur de droite en rotation par minute);
        asservie la vitesse de rotation des moteurs par un correcteur proportionnel-dérivé et commande les moteurs en conséquence;
        la variation de tension de commande des moteurs entre deux tics de calcul est seuillée à la valeur step_max"""
        rpm_left, rpm_right = self.get_rpm()

        # left motor
        e_left = rpm_left_bar - rpm_left
        self.ei_left += e_left * self.dt
        step_left = self.cst['left']['kp'] * e_left + self.cst['left']['ki'] * self.ei_left

        # right motor
        e_right = rpm_right_bar - (-rpm_right)
        self.ei_right += e_right * self.dt
        step_right = self.cst['right']['kp'] * e_right + self.cst['right']['ki'] * self.ei_right

        # On seuil la variation en tension
        if abs(step_left) > self.step_max:
            step_left = self.step_max * step_left / abs(step_left)
        if abs(step_right) > self.step_max:
            step_right = self.step_max * step_right / abs(step_right)

        self.cmd_left = max(min(self.u_max, self.cmd_left + step_left), 0)
        self.cmd_right = max(min(self.u_max, self.cmd_right + step_right), 0)

        self.ard.send_arduino_cmd_motor(self.cmd_left, self.cmd_right)

        # print('MEASURED RPM:', rpm_left, rpm_right)
        return rpm_left, rpm_right

    def regulation_proportion(self, rpm_left_bar, rpm_right_bar):
        rpm_left, rpm_right = self.get_rpm()

        # left motor
        e_left = rpm_left_bar - rpm_left
        self.ei_left += e_left * self.dt
        delta_e_left = (e_left - self.el_left) / self.dt
        self.el_left = e_left
        c_left = self.cst['left_']['kd'] * delta_e_left\
                 + self.cst['left_']['kp'] * e_left\
                 + self.cst['left_']['ki'] * self.ei_left

        # right motor
        e_right = rpm_right_bar - (-rpm_right)
        self.ei_right += e_right * self.dt
        delta_e_right = (e_right - self.el_right) / self.dt
        self.el_right = e_right
        c_right = self.cst['right_']['kd'] * delta_e_right\
                  + self.cst['right_']['kp'] * e_right\
                  + self.cst['right_']['ki'] * self.ei_right

        self.cmd_left = max(min(self.u_max, c_left), 0)
        self.cmd_right = max(min(self.u_max, c_right), 0)

        self.ard.send_arduino_cmd_motor(self.cmd_left, self.cmd_right)

        # print('MEASURED RPM:', rpm_left, rpm_right)
        return rpm_left, rpm_right

    def leo_cap_and_speed(self, delta_psi, rpm_max):
        """prend en argument un cap (dela_psi : angle avec le nord en degrés) et une vitesse de rotation maximale des moteurs (rpm_max : en rotation par minute); 
        renvoie les consignes en rotation par minute à donner aux moteurs pour que le bateau suive le cap demandé (ces valeurs ne dépasseront jamais la valeur entrée en paramètre (rmp_max))"""
        self.ei_psi += delta_psi * self.dt
        e_phi = self.cst['phi']['kp'] * delta_psi + self.cst['phi']['ki'] * self.ei_psi

        if e_phi >= 0:
            rpm_left_bar = rpm_max - e_phi * rpm_max
            rpm_right_bar = rpm_max
        else:
            rpm_right_bar = rpm_max + e_phi * rpm_max
            rpm_left_bar = rpm_max

        # print('RPM BAR:', rpm_left_bar, rpm_right_bar)
        return rpm_left_bar, rpm_right_bar

    def follow_psi(self, duration, psi_bar, speed_rpm):
        """prend en argument une durée de mission (duration : en seconde), un cap à suivre (psi_bar : angle avec le nord en degrés) et une vitesse de rotation maximale des moteurs (speed_rpm : en rotation par minute);
        écrit dans le fichier log les informations voulues (description de la mission en début d'execution puis à chaque boucle de calcul, le temps depuis le début de la mission,le cap mesuré par le bateau, la vitesse de rotation mesurée de chaque moteur, la consigne en rotation par minute donnée à chaque moteur et la température mesurée de chaque moteur);
        enregistre également à chaque boucle de calcul, la position gps du bateau;
        fait appelle aux méthodes d'asservissement du cap et de la rotation des moteurs pour faire se déplacer le bateau en ligne droite en suivant le cap entré en argument (psi_bar) pendant une durée (duration) sans que la consigne des moteurs ne dépasse la vitesse de rotation speed_rpm"""

        self.log.write("duration: %i ; psi_bar: %s ; spd: %i\n" % (duration, psi_bar, speed_rpm))

        self.reset()
        t0 = time.time()
        while (time.time() - t0) < duration:
            t0loop = time.time()

            psi = self.get_current_cap()
            delta_phi = sawtooth(psi_bar * (np.pi / 180) - psi)
            print("DELTA PHI: ", int(delta_phi * (180 / np.pi)))

            rpm_left_bar, rpm_right_bar = self.leo_cap_and_speed(delta_phi, speed_rpm)
            rpm_left, rpm_right = self.regulation_rpm(rpm_left_bar, rpm_right_bar)

            temp_left, temp_right = self.tpr.read_temp()
            data = [(t0loop - t0) * 1000, delta_phi * (180 / np.pi), rpm_left, rpm_right,
                    rpm_left_bar, rpm_right_bar, temp_left, temp_right]
            information = data_to_str(data)
            self.log.write(information)

            # print("Time left: ", self.dt - (time.time() - t0loop))
            while time.time() - t0loop < self.dt:
                self.gpsm.update_coord()
                time.sleep(0.001)

        self.ard.send_arduino_cmd_motor(0, 0)

    def follow_line(self, duration_max, line, speed_rpm):
        """prend en argument une durée maximale de mission (duration_max : en seconde), une ligne (line : du type line définit dans le fichier tools) et une vitesse de rotation maximale des moteurs (speed_rpm : en rotation par minute));
        fait appel à la méthode line_to_phi_bar pour obtenir un cap à suivre à chaque boucle de calcul afin de suivre au mieux la ligne entrée en paramètre (line) puis agit de la même manière que follow_psi pour suivre ce cap et remplir les logs;
        le bateau s'arrête lorsque la condition d'arrêt est remplie : l'objectif (définit par la position gps de la bouée visée dans line) est situé à 15 mètres ou moins du bateau, si cette condition n'est pas remplie au bout de duration_max, le bateau s'arrête"""

        self.log.write("duration_max: %i ; a: %s ; b: %s ; spd: %i\n" %
                       (duration_max, line.name0, line.name1, speed_rpm))

        self.reset()
        psi_bar = line.get_psi()
        t0 = time.time()
        cnt = 0
        while (time.time() - t0) < duration_max:  # TO ADD : ending conditions
            t0loop = time.time()

            # condition d'arrêt : distance à la bouée d'arrivée
            coord_boat = self.gpsm.coord
            pos_boat = coord_to_pos(coord_boat)
            dist = np.linalg.norm(line.pos1 - pos_boat)
            # print(dist)
            if dist <= 15:
                cnt += 1
                if cnt > 5:
                    break
            else:
                cnt = 0

            temp = self.line_to_phi_bar(line)
            psi_bar = temp if temp else psi_bar
            # print("PSI BAR: ", psi_bar * (180 / np.pi))

            psi = self.get_current_cap()
            delta_psi = sawtooth(psi_bar - psi)
            # print("DELTA PSI: ", int(delta_psi * (180 / np.pi)))

            rpm_left_bar, rpm_right_bar = self.leo_cap_and_speed(delta_psi, speed_rpm)
            rpm_left, rpm_right = self.regulation_rpm(rpm_left_bar, rpm_right_bar)
    
            temp_left, temp_right = self.tpr.read_temp()
            data = [(t0loop - t0) * 1000, delta_psi * (180 / np.pi), rpm_left, rpm_right,
                    rpm_left_bar, rpm_right_bar, temp_left, temp_right, pos_boat]
            information = data_to_str(data)
            self.log.write(information)

            # print("Time left: ", self.dt - (time.time() - t0loop))
            while time.time() - t0loop < self.dt:
                self.gpsm.update_coord()
                time.sleep(0.01)

        self.ard.send_arduino_cmd_motor(0, 0)

    def test_regulation_rpm(self, duration, rpm_spd, proportion=False):
        self.log.write("duration: %i ; spd: %i\n" % (duration, rpm_spd))

        self.reset()
        t0 = time.time()
        while (time.time() - t0) < duration:
            t0loop = time.time()

            if proportion:
                rpm_left, rpm_right = self.regulation_proportion(rpm_spd, rpm_spd)
            else:
                rpm_left, rpm_right = self.regulation_rpm(rpm_spd, rpm_spd)

            print(rpm_left, rpm_right)

            data = [(t0loop - t0) * 1000, rpm_left, rpm_right]
            information = data_to_str(data)
            self.log.write(information)

            while time.time() - t0loop < self.dt:
                time.sleep(0.01)

        self.ard.send_arduino_cmd_motor(0, 0)


if __name__ == '__main__':
    print("--- Control program ---\n")

    mn = input("Mission name: ")
    ctr = Control(mn)

    mt = input("Mission type (psi, square, line, test, triangle): ")

    if mt == 'line':
        a = input("Starting point: ")
        b = input("Ending point: ")
        my_line = Line(a, b)

        d_input = input("Mission max duration: ")
        d = math.inf if d_input == '' else int(d_input)

        s_input = input("Boat RPM speed: ")
        s = 3000 if s_input == '' else int(s_input)

        ctr.follow_line(d, my_line, speed_rpm=s)
    
    elif mt == 'triangle':
        #d_input = input("Test duration: ")
        #d = math.inf if d_input == '' else int(d_input)
        d = 200

        #s_input = input("Boat RPM speed: ")
        #s = 3000 if s_input == '' else int(s_input)
        s = 3000

        line1 = Line('ponton', 'ouest')
        line2 = Line('ouest', 'nord')
        line3 = Line('nord', 'ponton')
        ctr.follow_line(d, line1, speed_rpm=s)
        ctr.follow_line(d, line2, speed_rpm=s)
        ctr.follow_line(d, line3, speed_rpm=s)

    elif mt == 'square':
        d_input = input("Side duration: ")
        d = math.inf if d_input == '' else int(d_input)

        s_input = input("Boat RPM speed: ")
        s = 3000 if s_input == '' else int(s_input)

        ctr.follow_psi(d, speed_rpm=s, psi_bar=cap_to_psi('N'))
        ctr.follow_psi(d, speed_rpm=s, psi_bar=cap_to_psi('W'))
        ctr.follow_psi(d, speed_rpm=s, psi_bar=cap_to_psi('S'))
        ctr.follow_psi(d, speed_rpm=s, psi_bar=cap_to_psi('N'))

    elif mt == 'test':
        d_input = input("Test duration: ")
        d = math.inf if d_input == '' else int(d_input)

        s_input = input("Boat RPM speed: ")
        s = 3000 if s_input == '' else int(s_input)

        p_input = input("Proportion (0/1): ")
        p = (p_input == 1)

        ctr.test_regulation_rpm(d, rpm_spd=s, proportion=p)
    
    else:
        d_input = input("Mission duration: ")
        d = math.inf if d_input == '' else int(d_input)

        p_input = input("Psi bar: ")
        p = 0.0 if p_input == '' else int(p_input)

        s_input = input("Boat RPM speed: ")
        s = 3000 if s_input == '' else int(s_input)

        ctr.follow_psi(d, speed_rpm=s, psi_bar=p)

    ctr.close()
