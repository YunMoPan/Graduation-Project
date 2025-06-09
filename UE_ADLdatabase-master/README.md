# UE_ADLdatabase

created by Mia Huang

Committee members: Dr. Leia Bagesteiro, Dr. Charmaygne Hughes, Dr. Kate Hamel, Dr. Sandra Freitas

Institution affiliation: San Francisco State University

Date Created: Sep. 25th, 2021

Date Updated: Dec. 29th, 2021

## Overview

Open-access databases can facilitate data sharing among researchers worldwide. However, most existing databases focus on gait, balance, and hand gesture without providing elbow and shoulder kinematics that are required in activities of daily living. The lack of information hinders the robotic engineers’ ability to design robotic devices that accommodate the general population. To address the literature gap for the benefits of robotic designs as well as biomechanical modelling and mathematical investigation of human upper extremity, the purpose of this project is to develop an open-access upper limb kinematic database of daily tasks performance. The current database provides raw 3D marker trajectories and processed kinematic information of young, healthy adults performing high-priority ADLs, including forward reaching, reaching across, drinking, and combing hair. 

However, the COVID-19 pandemic has hindered the data collection progress, and a step-by-step instructional manual was developed as a guide for future data collection. Consequently, the current version of this database only includes 1 single participant as a feasibility check for the instructional manual and preliminary analysis. 

## Methods

### Participants

One 23-year-old female participant was tested and noted as ADL001. Nonetheless, the full database requires 158 healthy young adults between age 18-45 years with no upper extremity and trunk related neurological or orthopedic problem, no upper extremity injuries in the past six months, and normal or corrected-to-normal sight. 

### Experimental setup

A twelve-camera passive marker motion capture system (VICON Motion Systems, Oxford Metrics Ltd., Oxford, UK) was used to capture the movement at 100 Hz temporal resolution and 2mm spatial resolution. Ten Vicon Vero cameras (Oxford Metrics Ltd., Oxford, UK) and two Vicon MX13 cameras (Oxford Metrics Ltd., Oxford, UK) were used in this setup. The 3D coordinates of the reflective markers were reconstructed and labeled for each trial in Vicon Nexus software (v2.10). 

Marker model was adapted from Freitas and Scholtz (2009) with 17 individual markers and 8 clusters (Fig. 1). Additionally, one static calibration trial was recorded prior to the task performance (Fig. 2). 

![image](https://user-images.githubusercontent.com/85710749/134787271-a6a1ee5f-f7c1-4d97-b61a-ed58921ff1e1.png) 
![image](https://user-images.githubusercontent.com/85710749/134787303-66f7c57f-d3c6-4689-8644-199beb7af470.png)

Figure 1. Marker placements. 

![image](https://user-images.githubusercontent.com/85710749/147704485-639e8125-8112-43bc-a67d-5ae10ed0a210.png)

Figure 2. Static calibration position. 

#### ADL Tasks
##### Reaching Forward Task
Prior to the Reaching Forward task, participant was asked to reach forward as far as possible from the starting position without moving their trunk, and a round sticker was placed under the index finger to mark the position as the target (Roy, Moffet, & McFadyen, 2008). After the verbal cue, participant reached forward and touch the target with the tip of their index finger. The Reaching Straight task started with hands on the starting position and ended with index finger on the target. Participant performed five trials at comfortable speed with each arm (10 trials in total). 
##### Reaching Across Task
For the Reaching Across task, participant was asked to reach towards the front of their contralateral foot on the table until the elbow was extended without moving their trunk, and a round sticker was placed under the index finger to mark the position as the target (Roy, Moffet, & McFadyen, 2008). After the verbal cue, participant reached forward and touch the target with the tip of their index finger. The Reaching Across task started with hands on the starting position and ended with index finger on the target. Participant performed five trials at comfortable speed with one hand followed by the other hand (10 trials in total). 
##### Drinking Task
In the Drinking task, a non-compliant plastic cup (7cm in average diameter [bottom: 6cm, top: 8cm], 10cm in height) was placed directly in front of one hand (Beaudette & Chester, 2013). On the researcher’s voice command, participant would move their hand from the starting position (Fig. 3) to the cup, grasp the cup with extended fingers, transport the cup to the mouth, tilt the cup to simulate taking a drink, return the cup onto the target, and place their hand back on the starting position. The drinking task started with the hand on the starting position and ended when the hand returned to the starting position. Participant performed five trials on one side before the other (10 trials in total) as naturally as possible. 
##### Comb Hair Task
In the Comb Hair Task, participant was asked to place their palm on their forehead and move hand over the top of the head down to the neck as if they were combing their hair on the researcher’s voice command (Van Andel et al., 2008). Participant started and ended the Comb Hair task with both hands on the starting position. Participant was instructed to perform ten trials (five with the left hand, and five with the right hand) as naturally as possible without moving their trunk. 


![image](https://user-images.githubusercontent.com/85710749/147704577-3f834754-d0a3-42f8-80ad-767d525b24f3.png) 

Figure 3. Start position for all tasks.

## Results

The current dataset includes one meta file named ADLinfo.xlsx, 41 files with the raw marker position data, and 40 files with processed data. Naming convention for all data files are as demonstrated in Table 1. The processed files contained finger velocity and calculated joint angles, including shoulder girdle elevation, shoulder flexion/extension, abduction/adduction, internal/external rotation, elbow flexion/extension, forearm supination/pronation, wrist flexion/extension, ulnar/radial deviation. The processed files' onsets and offsets were determined using 3% finger velocity, and trajectories in between were filtered using a fourth-order, low-pass Butterworth filter with a cut-off frequency of 10 Hz.  

![image](https://user-images.githubusercontent.com/85710749/147704796-6aae3aa3-d12f-4912-8583-4ad671cb9cc0.png)

## License 

The data stored in this repository can be downloaded and are made available under the MIT License.  

## References

Beaudette, B. and Chester, V.L. (2013). Upper extremity kinematics in pediatric and young adult populations during activities of daily living. Journal of Medical and Biological Engineering, 34(5): 448-454. doi: 10.5405/jmbe.1697

Freitas, S.M.S.F. and Scholz, J.P. (2009). Does hand dominance affect the use of motor abundance when reaching to uncertain targets? Hum Mov Sci., 28(2), 169–190. doi:10.1016/j.humov.2009.01.003

Roy, J.S., Moffet, H., and McFadyen, B.J. (2008). Upper limb motor strategies in persons with and without shoulder impingement syndrome across different speeds of movement. Clinical Biomechanics, 23, 1227–1236. doi:10.1016/j.clinbiomech.2008.07.009

Van Andel, C. J., Wolterbeek, N., Doorenbosch, C.A.M., Veeger, D., and Harlaar, J. (2008). Complete 3D kinematics of upper extremity functional tasks. Gait & Posture, 27(1): 120-7. DOI: 10.1016/j.gaitpost.2007.03.002
