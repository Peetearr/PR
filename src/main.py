import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation
import argparse

class RightWall():
  def __init__(self, num):
    self.m = mujoco.MjModel.from_xml_path('model/labirint_'+num+'.xml')
    self.d = mujoco.MjData(self.m)
    mujoco.mj_step(self.m, self.d)

    self.error = 0
    self.prev_error = 0
    self.Kp = 10
    self.Kd = 15
    self.dt = .001

  def getdata(self):
    # данные с дальномеров
    left = self.d.sensor("lidar_l").data[0]
    right = self.d.sensor("lidar_r").data[0]
    left_45 = self.d.sensor("lidar_l_45").data[0]
    right_45 = self.d.sensor("lidar_r_45").data[0]
    forward = self.d.sensor("lidar_f").data[0]
    back = self.d.sensor("lidar_b").data[0]

    # данные с гиросокпа
    quaternion = self.d.xquat[1]
    rot = Rotation.from_quat(quaternion)
    euler_angles = rot.as_euler('XYZ', degrees=True)
    angle = euler_angles[0] + 180

    return [left, right, left_45, right_45, forward, back, angle]

  def sim(self):
    '''
    переменная flag исользуется для перехода на подпрограммы движения
    flag = 0: движение прямо
    flag = 1: поворот направо
    flag = 2: поворот налево
    flag = 3: разворот на 180
    '''
    flag = 0  
    flag_process = True  
    flag_init = True
    t_fin = 0
    with mujoco.viewer.launch_passive(self.m, self.d) as viewer:
      with mujoco.Renderer(self.m, 400, 600) as renderer:
          while True:
            data = self.getdata()
            self.data = data

            # расчет режима управления, в случае если развороты завершены
            if not flag_process:
              if (data[1] > .4 and data[3] > .4) or data[1] < 0:
                flag = 1
                angle_0 = data[6]
                self.prev_error = 0
                self.error = 0
                print('поворот направо')
              elif data[4] > .45 or data[4] < 0:
                flag = 0
                angle_0 = data[6]
              elif (data[0] > .4 and  data[2] > .4) or data[0] < 0:
                flag = 2
                angle_0 = data[6]
                self.prev_error = 0
                self.error = 0
                print('поворот налево')
              elif data[4] < .4:
                flag = 3
                angle_0 = data[6]
                self.prev_error = 0
                self.error = 0
                print('разворот на 180')

            # расчет режима управления, в случае если развороты завершены
            if flag_init:
              if not(data[4] > .45 and ((data[6]>88 and data[6]<92) or (data[6]>178 and data[6]<182) or \
               (data[6]>268 and data[6]<272) or (data[6]>358 or data[6]<2))):
                self.d.ctrl = [-.2, .2]
              else: 
                flag_init = False
                flag_process = False
            else:
              self.d.ctrl, flag_process = self.control(data[6], angle_0, flag)

            # проверка успешности прохождения лабиринта
            if self.d.qpos[0] > 3.5:
              print('успех')
              while t_fin < 10000:
                flag_process = True
                self.d.ctrl = [0,0]
                t_fin += 1
                mujoco.mj_step(self.m, self.d)
                viewer.sync()
              break
            mujoco.mj_step(self.m, self.d)
            viewer.sync()

  # метод ПД-регулятор
  def PD(self):
    self.error = .245 - self.data[1]
    u = self.Kp*self.error + self.Kd * (self.error - self.prev_error) / self.dt
    self.prev_error = self.error
    if  self.data[4] < .6 or self.data[5] < .6: u = 0
    # ограничение управляющего воздействия
    if u > .2: u = .2
    if u < -.2: u = -.2
    
    return u

  # метод расчета управляющего воздействия
  def control(self, angle, angle_0, flag):
    '''
    1. для прямолинейного движения используется ПД-регулятор по данным с правого дальномера;
    2. для поворотов и разворота на 180 регулирование по дальномеру останавливается до оконочания 
    разворота или поворота
    '''
    f_process = False
    if flag==0:
      u = self.PD()
      return [1-u, 1+u], f_process
    elif flag==1:
      # пересчет углов при переходе через значения 360->0
      if angle_0 > 350: angle_0 = angle_0 - 360
      if angle_0 > 270 and angle_0 <300 and angle < 270: angle += 360
      if angle_0 < 0 and angle > 180: angle -= 360

      if angle - angle_0 < 89.999: 
        f_process = True
      return [1.5, .5], f_process
    elif flag==2:
      # пересчет углов при переходе через значения 360->0
      if angle_0 < 10: angle_0 = angle_0 + 360
      if angle_0 < 90 and angle_0 > 70 and angle > 90: angle -= 360
      if angle_0 > 350 and angle < 180: angle += 360

      if angle_0 - angle < 89.999: 
        f_process = True
      return [.5, 1.5], f_process
    elif flag==3:
      if np.abs(angle_0 - angle) < 179: f_process = True
      return [-.5, .5], f_process
    
if __name__ == "__main__":
  # выбор номера лабиринта (1/2)
  parser = argparse.ArgumentParser()
  parser.add_argument('--num', default='1')
  args = parser.parse_args()

  robot = RightWall(args.num)
  robot.sim()
