# MIPS-AssemblyInverseKinematics

<img width="519" height="455" alt="Screenshot from 2026-02-18 23-33-20" src="https://github.com/user-attachments/assets/d8d627f5-9ac8-4882-9a56-59fdba4c91ec" />
This project is about doing inverse kinematics over a 2D robotic arm in MIPS ASSEMBLY for my Computer Organization and Architecture class.
It takes L1 and L2 inputs, which are the lengths of the parts of the arm, and Px and Py, which are the corresponding coordinates for the end-affector (end of the arm).
The problem is to determine Theta 1 and Theta 2, which are the angles that the joints of the robotic arm should have in relation to the X-Axis(Theta 2) and L1 itself(Theta 2) when the end-affector is in (Px, Py).
Those angles are the output, given by the cosine law, with some sofisticated adjustments and additional calculations, such as the Taylos Series, calculating acos, asin, atan, atan2 etc.
