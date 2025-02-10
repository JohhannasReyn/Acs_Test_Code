#!/usr/bin/env python
# coding: utf-8

# In[3]:


import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np
from scipy.stats import zscore


# In[2]:


'''
Defining function to apply softiron offsets depending on voltage
 '''

def calc_softirons(pwm_num, voltage, direction):

  voltage = round(voltage,1)
  if (voltage < 3.6): voltage = 3.6
  if (voltage > 4.2): voltage = 4.2

  # print('voltage: '+str(voltage))

  softirons = pd.read_csv(str(os.getcwd())+'/mag_calibration/softirons_'+direction+'.csv').drop(columns=['Unnamed: 0'])
  # softirons = softirons['3.6v']
  softirons = softirons[str(voltage)+'v']

  pwmY_ox_1 = softirons[0];
  pwmY_ox_2 = softirons[1];
  pwmY_ox_3 = softirons[2];
  pwmY_oy_1 = softirons[3];
  pwmY_oy_2 = softirons[4];
  pwmY_oy_3 = softirons[5];
  pwmY_oz_1 = softirons[6];
  pwmY_oz_2 = softirons[7];
  pwmY_oz_3 = softirons[8];

  pwmY_ox = (pwmY_ox_1 * pwm_num) + (pwmY_ox_2 * pow(pwm_num, 2)) + (pwmY_ox_3 * pow(pwm_num, 3));
  pwmY_oy = (pwmY_oy_1 * pwm_num) + (pwmY_oy_2 * pow(pwm_num, 2)) + (pwmY_oy_3 * pow(pwm_num, 3));
  pwmY_oz = (pwmY_oz_1 * pwm_num) + (pwmY_oz_2 * pow(pwm_num, 2)) + (pwmY_oz_3 * pow(pwm_num, 3));

  return pwmY_ox, pwmY_oy, pwmY_oz


# In[ ]:


'''
Processsing testing data for the CubeSat, creating a dataset, not plotting code
 '''

# plt.figure(figsize=(6,4), dpi=200)
import_data = pd.read_csv('cubesat_imus.csv').drop(columns='Unnamed: 0')
direction = 'z'

for num in np.arange(3.6, 4.2, .1): 

  data = pd.read_csv(str(os.getcwd())+'/data/10.23.2024_voltimu_dataset/cubesat_voltimu_'+str(round(num,1))+'v.txt', names=['values'])
  # data_off = pd.read_csv(str(os.getcwd())+'/data/10.22.2024_voltimu_dataset/cubesat_crazytest.txt', names=['values'])
  # data = pd.read_csv(str(os.getcwd())+'/data/10.23.2024_voltimu_dataset/cubesat_voltimu_4.0v_'+str(i)+'.txt', names=['values'])
  # print('num: '+str(num))

  pwms = []
  mag_x = []
  offsets_x = []
  mag_y = []
  offsets_y = []
  mag_z = []
  offsets_z = []
  voltage = []

  for i in range(0,len(data), 6):
    # pwm.append(float(data.loc[i+1]))
    pwm = float(data.loc[i+1])
    pwms.append(pwm)
    offset_x, offset_y, offset_z = calc_softirons(pwm, round(float(data.loc[i+5]), 1), direction)

    offsets_x.append(offset_x)
    offsets_y.append(offset_y)
    offsets_z.append(offset_z)

    mag_x.append(float(data.loc[i+2]))
    # mag_y.append(float(data.loc[i+3]))
    mag_y.append(float(data.loc[i+3]))
    mag_z.append(float(data.loc[i+4]))
    voltage.append(float(data.loc[i+5]))

  # mag_y = pd.Series(mag_y)
  # offset_y = pd.Series(offset_y)
  # voltage = pd.Series(voltage)

  num = str(round(num,2))
  if num == '3.6': 
    output_data = pd.DataFrame([pwms, mag_x, offsets_x, mag_y, offsets_y, mag_z, offsets_z, voltage]).T
    output_data.rename(columns={0:'pwm', 1:direction+'torq_xmag_'+num, 2:direction+'torq_xoffset_'+num, 3:direction+'torq_ymag_'+num, 4:direction+'torq_yoffset_'+num, 5:direction+'torq_zmag_'+num, 6:direction+'torq_zoffset_'+num, 7:direction+'torq_volt_'+num},inplace=True)
  else:
    output_data[direction+'torq_xmag_'+num] = mag_x
    output_data[direction+'torq_xoffset_'+num] = offsets_x
    output_data[direction+'torq_ymag_'+num] = mag_y
    output_data[direction+'torq_yoffset_'+num] = offsets_y
    output_data[direction+'torq_zmag_'+num] = mag_z
    output_data[direction+'torq_zoffset_'+num] = offsets_z
    output_data[direction+'torq_volt_'+num] = voltage

output_data.drop(columns='pwm', inplace=True)
output_data = pd.concat([import_data, output_data], axis=1)
output_data
output_data.to_csv('cubesat_imus.csv')

#   # plt.plot(np.arange(0, len(data)/7), pwm, label='PWM')
#   plt.plot(np.arange(0, len(data)/6), mag_x, label=str(round(num,1)))
#   plt.plot(np.arange(0, len(data)/6), mag_y, label=str(round(num,1)))
#   plt.plot(np.arange(0, len(data)/6), mag_z, label=str(round(num,1)))
#   # plt.plot(np.arange(0, len(data)/7), voltage, label='voltage')
#   plt.xlabel("indexes")
#   plt.ylabel('uT')
#   plt.legend(bbox_to_anchor=[1,1])

# # data = pd.DataFrame([mag_y,offset_y,voltage], columns=['mag_y','offset_y','voltage'])
# plt.show()


# In[14]:


'''
Processsing testing data for the CubeSat (same PWM, varying voltage with dial)
 '''

plt.figure(figsize=(6,4), dpi=200)

for num in np.arange(3.6, 4.0, .1): 

  data = pd.read_csv(str(os.getcwd())+'/data/11.14.2024_voltimu_dataset/cubesat_xtorq_'+str(round(num,1))+'v.txt', names=['values'])
  # data = pd.read_csv(str(os.getcwd())+'/data/10.22.2024_voltimu_dataset/cubesat_crazytest.txt', names=['values'])
  # data = pd.read_csv(str(os.getcwd())+'/data/10.23.2024_voltimu_dataset/cubesat_voltimu_4.0v_'+str(i)+'.txt', names=['values'])
  # print('num: '+str(num))

  mag_x = []
  mag_y = []
  mag_z = []
  voltage = []

  for i in range(0,len(data), 6):
    # pwm.append(float(data.loc[i+1]))
    pwm = float(data.loc[i+1])
    # offset_x, offset_y, offset_z = calc_softirons(pwm, round(float(data.loc[i+5]), 1), direction)

    # offsets_x.append(offset_x)
    # offsets_y.append(offset_y)
    # offsets_z.append(offset_z)

    mag_x.append(float(data.loc[i+2]))
    # mag_y.append(float(data.loc[i+3]))
    mag_y.append(float(data.loc[i+3]))
    mag_z.append(float(data.loc[i+4]))
    voltage.append(float(data.loc[i+5]))

  # mag_y = pd.Series(mag_y)
  # offset_y = pd.Series(offset_y)
  # voltage = pd.Series(voltage)

  # plt.plot(np.arange(0, len(data)/7), pwm, label='PWM')
  plt.plot(np.arange(0, len(data)/6), mag_x, label=str(round(num,1)))
  plt.plot(np.arange(0, len(data)/6), mag_y, label=str(round(num,1)))
  plt.plot(np.arange(0, len(data)/6), mag_z, label=str(round(num,1)))
  # plt.plot(np.arange(0, len(data)/7), voltage, label='voltage')
  plt.xlabel("indexes")
  plt.ylabel('uT')
  plt.legend(bbox_to_anchor=[1,1])

# # data = pd.DataFrame([mag_y,offset_y,voltage], columns=['mag_y','offset_y','voltage'])
# plt.show()


# In[21]:


data = pd.read_csv('cubesat_imus.csv').drop(columns='Unnamed: 0')
data_simple = pd.read_csv('cubesat_imus_simple.csv').drop(columns='Unnamed: 0')

dirs = ['x','y','z']

for torq in dirs:
  for mag in dirs:
# torq='y'
# mag='z'
    plt.figure(figsize=(6,4), dpi=200)
    plt.title(torq+' magnetorquer and '+mag+' magnetometer')
    plt.ylabel('magnetic strength (uT)')
    plt.xlabel('PWM')
    for i in np.arange(3.6, 4.2, 0.1):
      i = str(round(i,2))

      # plt.plot(data['pwm'], data[torq+'torq_'+mag+'mag_'+i], label=i, color='green')
      plt.plot(data['pwm'], data[torq+'torq_'+mag+'mag_'+i] - data[torq+'torq_'+mag+'offset_'+i], label=i, color='red')
      plt.plot(data['pwm'], data[torq+'torq_'+mag+'mag_'+i] - data_simple[torq+'torq_'+mag+'offset_'+i], label=i, color='blue')
  
    plt.savefig(torq+'torq_'+mag+'mag', dpi=200)

# plt.legend()
# plt.plot(data['pwm'], data['xtorq_ymag']-data['xtorq_yoffset'])


# In[14]:


'''
Processsing testing data for the breadboard (same PWM, varying voltage with dial)
 '''

data = pd.read_csv(str(os.getcwd())+'/data/test.txt', header=1, index_col = False, names=['pwm', 'mag_x', 'mag_y', 'mag_z', 'offset_x', 'offset_y', 'offset_z', 'voltage'])
data

# plt.figure(figsize=(6,4), dpi=200)
# plt.plot(pd.Series(data.index), zscore(data.mag_y-data.offset_y))
# plt.plot(pd.Series(data.index), zscore(data.voltage))
# plt.xlabel("indexes")

plt.figure(figsize=(6,4), dpi=200)
plt.plot(pd.Series(data.index), data.mag_y)
plt.xlabel("indexes")
plt.ylabel('uT')

plt.figure(figsize=(6,4), dpi=200)
plt.plot(pd.Series(data.index), data.voltage)
plt.xlabel("indexes")
plt.ylabel('V')


# In[22]:


path = str(os.getcwd())+"/data/10.17.2024_voltimu_dataset/"
# data = pd.read_csv(path+'bread_voltimu_3.6v.txt', header=1, index_col = False, names=['pwm', 'mag_x', 'mag_y', 'mag_z', 'offset_x', 'offset_y', 'offset_z', 'voltage'])
# data['mag_y'].head(1)


# In[9]:


'''
Processsing testing data for the breadboard (vary PWM, same voltage)
 '''

path = str(os.getcwd())+"/data/10.17.2024_voltimu_dataset/"

slopes = []

pwmY_ox_1 = 8.60013482e-03;
pwmY_ox_2 = -2.04542449e-06;
pwmY_ox_3 = 6.14475215e-09;
pwmY_oy_1 = -2.37677372e-01;
pwmY_oy_2 = -3.66292599e-07;
pwmY_oy_3 = 7.25220153e-08;
pwmY_oz_1 = -1.47995640e-02;
pwmY_oz_2 = 1.97761126e-07;
pwmY_oz_3 = 2.85273223e-08;

plt.figure(figsize=(6,4), dpi=200)
for i in np.arange(3.6, 4.2, .1): 
  # print(i)
  data = pd.read_csv(path+'bread_voltimu_'+str(round(i,2))+'v.txt', header=1, index_col = False, names=['pwm', 'mag_x', 'mag_y', 'mag_z', 'offset_x', 'offset_y', 'offset_z', 'voltage'])

  # data_2 = pd.read_csv(path+'voltimugraph_'+str(round(i,2))+'v_test2.txt', header=1, index_col = False, names=['pwm', 'mag_x', 'mag_y', 'mag_z', 'offset_x', 'offset_y', 'offset_z', 'voltage'])
  slopes.append(-(float(data['mag_y'].head(1)) - float(data['mag_y'].tail(1)))/510)
  voltage = data['voltage'].mean()
  # voltage_2 = data_2['voltage'].mean()
  
  # data.offset_y = (pwmY_oy_1 * data.pwm) + (pwmY_oy_2 * pow(data.pwm, 2)) + (pwmY_oy_3 * pow(data.pwm, 3))

  print(data.voltage.mean())
  offset_x, offset_y, offset_z = calc_softirons(data.pwm, data.voltage.mean())

  plt.plot(pd.Series(data.index), data.mag_y - offset_y, label = str(round(voltage,2)))
  # plt.plot(pd.Series(data.index), data.mag_y, label = str(round(voltage,2)))
  # plt.plot(pd.Series(data_2.index), data_2.mag_y - data_2.offset_y, label = str(round(voltage_2,2)))
  # plt.plot(pd.Series(data.index), data.mag_y, label = str(round(voltage,2)))
  plt.xlabel("indexes")
  plt.ylabel("uT")

# data = pd.read_csv(path+'voltimugraph_4.0v_slow.txt', header=1, index_col = False, names=['pwm', 'mag_x', 'mag_y', 'mag_z', 'offset_x', 'offset_y', 'offset_z', 'voltage'])
# plt.plot(pd.Series(data.index)*10, data.mag_y, label = '4.0v_slow')
plt.legend(bbox_to_anchor=[1, 1])

plt.figure(figsize=(6,4), dpi=200)
plt.plot(pd.Series(data.index), data.voltage)

# mag=[]
# volt=[]
# plt.figure(figsize=(6,4), dpi=200)
# for i in np.arange(3.60, 4.2, .05): 
#   data = pd.read_csv(path+'voltimugraph_'+str(round(i,2))+'v.txt', header=1, index_col = False, names=['pwm', 'mag_x', 'mag_y', 'mag_z', 'offset_x', 'offset_y', 'offset_z', 'voltage'])
#   mag.append(data['mag_y'][250])
#   volt.append(data['voltage'].mean())

# plt.scatter(volt, mag)
# plt.xlabel("voltage")
# plt.ylabel("uT")

plt.figure(dpi=200)
plt.plot(np.arange(3.6, 4.2, .1), slopes)
(slopes[-1] - slopes[0]) / 0.6


# In[5]:


# data = pd.read_csv(str(os.getcwd())+'/data/airbearingrig_tests/test28_oct2024.txt', header=1, index_col = False, names=['error', 'current', 'PWM', 'mag_x', 'mag_z', 'mag_y', 'battery'])

# data_off = pd.read_csv(str(os.getcwd())+'/data/airbearingrig_tests/test66_dec2024.txt', header=0, index_col = False, names=['error', 'current', 'PWM', 'mag_x', 'mag_y', 'mag_z', 'hardiron_x', 'hardiron_y', 'hardiron_z', 'softiron_x', 'softiron_y', 'softiron_z', 'voltage'])
data_on = pd.read_csv(str(os.getcwd())+'/data/airbearingrig_tests/test66_dec2024.txt', header=0, index_col = False, names=['error', 'current', 'PWM', 'mag_x', 'mag_y', 'mag_z', 'hardiron_x', 'hardiron_y', 'hardiron_z', 'softiron_x', 'softiron_y', 'softiron_z', 'voltage'])

# data_on = pd.read_csv(str(os.getcwd())+'/data/airbearingrig_tests/test26_oct2024.txt', header=0, index_col = False, names=['error', 'current', 'PWM', 'mag_x', 'mag_z', 'mag_y', 'voltage'])

# data = pd.read_csv(str(os.getcwd())+'/data/test23_sep2-024.txt', header=1, index_col = False, names=['pwm', 'mag_x', 'mag_y', 'mag_z', 'offset_x', 'offset_y', 'offset_z', 'voltage'])
# data_off


# In[11]:


# plt.figure(figsize=(6,4), dpi=200)
# plt.plot(pd.Series(data.index), data.PWM)
# # plt.title("Test 17 PWM values")
# plt.xlabel("indexes")
# plt.ylabel("PWM")

# plt.figure(figsize=(6,4), dpi=200)
# plt.plot(pd.Series(data_off.index)*0.1, data_off.mag_x, label='x')
# plt.plot(pd.Series(data_off.index)*0.1, data_off.mag_y, label='y')
# plt.plot(pd.Series(data_off.index)*0.1, data_off.mag_z, label='z')
# # plt.plot(pd.Series(data.index), data.mag_x - data.offset_x, color='blue')
# # plt.plot(pd.Series(data.index), data.mag_y - data.offset_y, color='blue')
# # plt.plot(pd.Series(data.index), data.mag_z - data.offset_z, color='blue')
# # plt.title("Test 17 Mag values") 
# plt.xlabel("sec")
# plt.ylabel("uT")
# plt.legend()

# plt.figure(figsize=(6,4), dpi=200)
# plt.plot(pd.Series(data_off.index)*0.1, data_off.PWM)
# plt.xlabel("sec")
# plt.ylabel("PWM")

plt.figure(figsize=(6,4), dpi=200)
# plt.plot(pd.Series(data_on.index)*0.1, data_on.mag_x - data_on.hardiron_x - data_on.softiron_x, label='x')
# plt.plot(pd.Series(data_on.index)*0.1, data_on.mag_y - data_on.hardiron_y - data_on.softiron_y, label='y')
# plt.plot(pd.Series(data_on.index)*0.1, data_on.mag_z - data_on.hardiron_z - data_on.softiron_z, label='z')
# plt.plot(pd.Series(data_on.index)*0.1, data_on.mag_x, label='x')
plt.plot(pd.Series(data_on.index)*0.1, data_on.mag_y, label='y')
# plt.plot(pd.Series(data_on.index)*0.1, data_on.mag_z, label='z')
plt.xlabel("sec")
plt.xlim(80,90)
plt.ylabel("uT")
plt.legend()

plt.figure(figsize=(6,4), dpi=200)
plt.plot(pd.Series(data_on.index)*0.1, data_on.PWM)
plt.xlabel("sec")
plt.xlim(80,90)
plt.ylabel("PWM")

# plt.figure(figsize=(6,4), dpi=200)
# plt.plot(pd.Series(data.index), data.battery)


# In[29]:


# data = data.iloc[2000:4000]


data_off['angle'] = np.arctan(data_off.mag_x/data_off.mag_y) * 180 / np.pi
# data['ang_velocity'] = np.gradient(data['angle'])
# data['ang_accel'] = np.gradient(data['ang_velocity'])

plt.figure(figsize=(6,4), dpi=200)
plt.plot(pd.Series(data_off.index)*.1, data_off.angle, label='angle', color='black')
# plt.plot(pd.Series(data.index), data.PWM, label='pwm')
# plt.plot(pd.Series(data.index), data.ang_velocity)
# plt.plot(pd.Series(data.index), data.ang_accel)
plt.xlabel("sec")
plt.ylabel("deg")
plt.legend()
# plt.ylabel("deg")
# plt.ylim((-5,5))

plt.figure(figsize=(6,4), dpi=200)
plt.plot(pd.Series(data_off.index)*0.1, data_off.PWM, color='r')
plt.xlabel("sec")
plt.ylabel("PWM")

data_on['angle'] = np.arctan(data_on.mag_x/data_on.mag_y) * 180 / np.pi
plt.figure(figsize=(6,4), dpi=200)
plt.plot(pd.Series(data_on.index)*.1, data_on.angle, label='angle', color='black')
plt.xlabel("sec")
plt.ylabel("deg")
plt.legend()

plt.figure(figsize=(6,4), dpi=200)
plt.plot(pd.Series(data_on.index)*0.1, data_on.PWM, color='r')
plt.xlabel("sec")
plt.ylabel("PWM")


# In[ ]:





# In[ ]:




