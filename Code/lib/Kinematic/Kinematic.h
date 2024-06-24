#ifndef _kinematic_h
#define _kinematic_h
#include <Arduino.h>
#include <math.h>

// Structure to hold DH parameters for each joint
struct DHParameters {
    float d;     // Link offset (along previous z-axis)
    float a;     // Link length (along x-axis)
    float alpha; // Link twist (about x-axis)
    float theta; // Joint angle (about z-axis)
};

class RobotArm {
public:
    RobotArm(int numJoints) {
        this->numJoints = numJoints;
        this->jointParams = new DHParameters[numJoints];
        this->jointMinAngles = new float[numJoints];
        this->jointMaxAngles = new float[numJoints];

        // Initialize joint angle limits (default: -90 to 90 degrees)
        for (int i = 0; i < numJoints; i++) {
            jointMinAngles[i] = -90.0;
            jointMaxAngles[i] = 90.0;
        }
    }

    ~RobotArm() {
        delete[] jointParams;
        delete[] jointMinAngles;
        delete[] jointMaxAngles;
    }

    // Set DH parameters for a specific joint
    void setDHParameters(int jointIndex, float a, float alpha, float d, float theta) {
        if (jointIndex >= 0 && jointIndex < numJoints) {
            jointParams[jointIndex].d = d;
            jointParams[jointIndex].a = a;
            jointParams[jointIndex].alpha = alpha;
            jointParams[jointIndex].theta = theta;
        }
    }

    // Set joint angle limits for a specific joint
    void setJointAngleLimits(int jointIndex, float minAngle, float maxAngle) {
        if (jointIndex >= 0 && jointIndex < numJoints) {
            jointMinAngles[jointIndex] = minAngle;
            jointMaxAngles[jointIndex] = maxAngle;
        }
    }

    // Compute forward kinematics to get end-effector pose
    void computeForwardKinematics(float* jointAngles, float* eePose) {
        if (jointAngles == nullptr || eePose == nullptr) return;

        // Initialize transformation matrix as identity (4x4)
        float T[4][4] = {{1.0, 0.0, 0.0, 0.0},
                         {0.0, 1.0, 0.0, 0.0},
                         {0.0, 0.0, 1.0, 0.0},
                         {0.0, 0.0, 0.0, 1.0}};

        // Compute transformation matrix using DH parameters
        for (int i = 0; i < numJoints; i++) {
            float d = jointParams[i].d;
            float a = jointParams[i].a;
            float alpha = jointParams[i].alpha * DEG_TO_RAD;
            float theta = jointAngles[i] * DEG_TO_RAD; // Convert to radians

            float cosTheta = cos(theta);
            float sinTheta = sin(theta);
            float cosAlpha = cos(alpha);
            float sinAlpha = sin(alpha);

            // Construct transformation matrix for current joint
            float A[4][4] = {{cosTheta, -sinTheta * cosAlpha, sinTheta * sinAlpha, a * cosTheta},
                             {sinTheta, cosTheta * cosAlpha, -cosTheta * sinAlpha, a * sinTheta},
                             {0.0, sinAlpha, cosAlpha, d},
                             {0.0, 0.0, 0.0, 1.0}};

            // Multiply current transformation with new transformation
            matrixMultiply(T, A);
        }

        // Extract end-effector pose (position and orientation)
        eePose[0] = T[0][3]; // X position
        eePose[1] = T[1][3]; // Y position
        eePose[2] = T[2][3]; // Z position

        // Extract orientation (assuming rotation matrix)
        eePose[3] = atan2(T[2][1], T[2][2]) * RAD_TO_DEG;             // Roll (around X-axis)
        eePose[4] = atan2(-T[2][0], sqrt(T[2][1]*T[2][1] + T[2][2]*T[2][2])) * RAD_TO_DEG; // Pitch (around Y-axis)
        eePose[5] = atan2(T[1][0], T[0][0]) * RAD_TO_DEG;  
    }

private:
    int numJoints;
    DHParameters* jointParams;
    float* jointMinAngles;
    float* jointMaxAngles;

    // Matrix multiplication (4x4 matrix)
    void matrixMultiply(float A[4][4], float B[4][4]) {
        float tmp[4][4];

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                tmp[i][j] = 0.0;
                for (int k = 0; k < 4; k++) {
                    tmp[i][j] += A[i][k] * B[k][j];
                }
            }
        }

        // Copy result back to matrix A
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                A[i][j] = tmp[i][j];
            }
        }
    }
};

RobotArm setup_kin()
{

    RobotArm arm(6);  // Create a robotic arm with 4 joints

    // Set DH parameters for each joint
    arm.setDHParameters(0, 0.0, 90.0, 94.85, 0.0);  // Joint 1
    arm.setDHParameters(1, 104, 0.0, 0.0, -90.0);  // Joint 2
    arm.setDHParameters(2, 104, 0, 0.0, 180.0);   // Joint 3
    arm.setDHParameters(3, 0.0, 90.0, -7.65, 0.0);   // Joint 4
    arm.setDHParameters(4, 0.0, -90.0, 66.6, 0.0);   // Joint 4
    arm.setDHParameters(5, 0.0, 0.0, 66.6, 0.0);   // Joint 4

    // Set joint angle limits for each joint
    arm.setJointAngleLimits(0, -90.0, 90.0);  // Joint 1
    arm.setJointAngleLimits(1, -90.0, 90.0);  // Joint 2
    arm.setJointAngleLimits(2, -90.0, 90.0);  // Joint 3
    arm.setJointAngleLimits(3, -90.0, 90.0);  // Joint 4
    arm.setJointAngleLimits(4, -90.0, 90.0);  // Joint 5
    arm.setJointAngleLimits(5, -90.0, 90.0);  // Joint 6
    return arm;

}


void use_example(RobotArm& arm)
{
    
    float jointAngles[] = {90, 90.0, 0, 90, 0, 0}; // Joint angles in degrees
    float eePose[6]; // End-effector pose (x, y, z, roll, pitch, yaw)

    // Compute forward kinematics
    arm.computeForwardKinematics(jointAngles, eePose);
    // Print end-effector pose
    Serial.println("End-Effector Pose:");
    Serial.print("Position (XYZ): ");
    Serial.print(eePose[0]);
    Serial.print(", ");
    Serial.print(eePose[1]);
    Serial.print(", ");
    Serial.println(eePose[2]);
    Serial.print("Orientation (RPY): ");
    Serial.print(eePose[3]);
    Serial.print(", ");
    Serial.print(eePose[4]);
    Serial.print(", ");
    Serial.println(eePose[5]);
}
#endif



// #include <Arduino.h>
// #include <math.h>
// #include "Matrix.h"
// using RixMatrix::Matrix;

// struct DHParameters {
//     float d;     // Link offset (along previous z-axis)
//     float a;     // Link length (along x-axis)
//     float alpha; // Link twist (about x-axis)
//     float theta; // Joint angle (about z-axis)
// };

// struct EEPos
// {
//     float jointAngles[6];
//     float eePose[6]; 
// };

// class RobotArm {
// public:
//     RobotArm() {
//         // this->numJoints = numJoints;
//         this->jointParams = new DHParameters[numJoints];
//         this->jointMinAngles = new float[numJoints];
//         this->jointMaxAngles = new float[numJoints];
//         this->ee_pos = new EEPos;
//         // Initialize joint angle limits (default: -90 to 90 degrees)
//         for (int i = 0; i < numJoints; i++) setJointAngleLimits(i, -90, 90);

//     }

//     ~RobotArm() {
//         delete[] jointParams;
//         delete[] jointMinAngles;
//         delete[] jointMaxAngles;
//         delete[] ee_pos;
//     }

//     void setDHParameters(int jointIndex, float a, float alpha, float d, float theta) {
//         if (jointIndex >= 0 && jointIndex < numJoints) 
//         {
//             jointParams[jointIndex].d = d;
//             jointParams[jointIndex].a = a;
//             jointParams[jointIndex].alpha = alpha;
//             jointParams[jointIndex].theta = theta;
//         }
//     }

//     void setJointAngleLimits(int jointIndex, float minAngle, float maxAngle) {
//         if (jointIndex >= 0 && jointIndex < numJoints) 
//         {
//             jointMinAngles[jointIndex] = minAngle;
//             jointMaxAngles[jointIndex] = maxAngle;
//         }
//     }

//     void computeForwardKinematics(float* jointAngles) {
//         if (jointAngles == nullptr) return;
//         // Initialize transformation matrix as identity (4x4)
//         float T[4][4] = {{1.0, 0.0, 0.0, 0.0},
//                          {0.0, 1.0, 0.0, 0.0},
//                          {0.0, 0.0, 1.0, 0.0},
//                          {0.0, 0.0, 0.0, 1.0}};

//         // Compute transformation matrix using DH parameters
//         for (int i = 0; i < numJoints; i++) 
//         {
//             float d = jointParams[i].d;
//             float a = jointParams[i].a;
//             float alpha = jointParams[i].alpha * DEG_TO_RAD;
//             float theta = jointAngles[i] * DEG_TO_RAD; // Convert to radians

//             float cosTheta = cos(theta);
//             float sinTheta = sin(theta);
//             float cosAlpha = cos(alpha);
//             float sinAlpha = sin(alpha);

//             // Construct transformation matrix for current joint
//             float A[4][4] = {{cosTheta, -sinTheta * cosAlpha, sinTheta * sinAlpha, a * cosTheta},
//                              {sinTheta, cosTheta * cosAlpha, -cosTheta * sinAlpha, a * sinTheta},
//                              {0.0, sinAlpha, cosAlpha, d},
//                              {0.0, 0.0, 0.0, 1.0}};

//             // Multiply current transformation with new transformation
//             matrixMultiply(T, A);
//         }

//         // Extract end-effector pose (position and orientation)
//         ee_pos->eePose[0] = T[0][3]; // X position
//         ee_pos->eePose[1] = T[1][3]; // Y position
//         ee_pos->eePose[2] = T[2][3]; // Z position
//         // Extract orientation (assuming rotation matrix)
//         ee_pos->eePose[3] = atan2(T[2][1], T[2][2]) * RAD_TO_DEG;             // Roll (around X-axis)
//         ee_pos->eePose[4] = atan2(-T[2][0], sqrt(T[2][1]*T[2][1] + T[2][2]*T[2][2])) * RAD_TO_DEG; // Pitch (around Y-axis)
//         ee_pos->eePose[5] = atan2(T[1][0], T[0][0]) * RAD_TO_DEG;  

//         for (int i=0; i <6; i++)
//             ee_pos->jointAngles[i] = jointAngles[i];
//     }

//     void computeForwardKinematics(float* jointAngles, float* eePose) {
//         if (jointAngles == nullptr) return;
        
//         // Initialize transformation matrix as identity (4x4)
//         float T[4][4] = {{1.0, 0.0, 0.0, 0.0},
//                             {0.0, 1.0, 0.0, 0.0},
//                             {0.0, 0.0, 1.0, 0.0},
//                             {0.0, 0.0, 0.0, 1.0}};

//         // Compute transformation matrix using DH parameters
//         for (int i = 0; i < numJoints; i++) 
//         {
//             float d = jointParams[i].d;
//             float a = jointParams[i].a;
//             float alpha = jointParams[i].alpha * DEG_TO_RAD;
//             float theta = jointAngles[i] * DEG_TO_RAD; // Convert to radians

//             float cosTheta = cos(theta);
//             float sinTheta = sin(theta);
//             float cosAlpha = cos(alpha);
//             float sinAlpha = sin(alpha);

//             // Construct transformation matrix for current joint
//             float A[4][4] = {{cosTheta, -sinTheta * cosAlpha, sinTheta * sinAlpha, a * cosTheta},
//                                 {sinTheta, cosTheta * cosAlpha, -cosTheta * sinAlpha, a * sinTheta},
//                                 {0.0, sinAlpha, cosAlpha, d},
//                                 {0.0, 0.0, 0.0, 1.0}};

//             // Multiply current transformation with new transformation
//             matrixMultiply(T, A);
//         }

//         // Extract end-effector pose (position and orientation)
//         eePose[0] = T[0][3]; // X position
//         eePose[1] = T[1][3]; // Y position
//         eePose[2] = T[2][3]; // Z position
//         // Extract orientation (assuming rotation matrix)
//         eePose[3] = atan2(T[2][1], T[2][2]) * RAD_TO_DEG;             // Roll (around X-axis)
//         eePose[4] = atan2(-T[2][0], sqrt(T[2][1]*T[2][1] + T[2][2]*T[2][2])) * RAD_TO_DEG; // Pitch (around Y-axis)
//         eePose[5] = atan2(T[1][0], T[0][0]) * RAD_TO_DEG;  
//     }

//     bool computeInverseKinematics(float* desiredPose, float* jointAngles, const float tolerance=0.1, const int maxIterations=2)
//      {
//         if (desiredPose == nullptr || jointAngles == nullptr) return false;
//         Serial.print("Desired Pos:");
//         for(int i = 0; i < 6; i++)  
//         {
//             Serial.print(desiredPose[i]);
//             Serial.print(", ");
//         }
//         Serial.println();
        
//         float currentPose[6]; // Current end-effector pose
//         float error[6]; // Error between current and desired poses

//         // Initialize joint angles (e.g., start with zero configuration)
//         for (int i = 0; i < numJoints; i++) 
//             jointAngles[i] = ee_pos->jointAngles[i]; // Initial guess (Current angles)
        
//         // Iterative IK solver using Jacobian-based method
//         for (int iter = 0; iter < maxIterations; iter++) 
//         {
//             if (iter % 5 == 0)
//             {
//                 Serial.print("iteration:");
//                 Serial.println(iter);
//                 // Serial.print("Error:");
//                 // for(int i = 0; i < 6; i++)  
//                 //     Serial.println(error[i]);
//             }

//             // Compute current end-effector pose using current joint angles                
//             computeForwardKinematics(jointAngles, currentPose);

//             // Compute error (difference) between current and desired poses
//             for (int i = 0; i < 6; i++) 
//                 error[i] = desiredPose[i] - currentPose[i]; // Position error
              
//             // Check convergence criteria
//             bool converged = true;
//             for (int i = 0; i < 6; i++) 
//             {
//                 if (isnan(error[i]) || fabs(error[i]) >= tolerance) 
//                 {
//                     converged = false;
//                     break;
//                 }
//             }

//             if (converged)  
//               {
//                 Serial.print("Found in iteration:");
//                 Serial.println(iter);
               
//                 for (int i = 0; i < numJoints; i++) 
//                 {
//                     Serial.print("angle bedore norm:");
//                     Serial.println(jointAngles[i]);
//                     jointAngles[i] = normalizeAngle(jointAngles[i]);
//                     Serial.print("Error:");
//                     Serial.println(error[i]);
//                 }
//                 return true; // IK converged
//               }
              
//             // Compute Jacobian matrix (partial derivatives of end-effector position w.r.t joint angles)
//             float jacobian[numJoints][6]; // Jacobian matrix (numJoints x 3)
//             // Compute Jacobian matrix (example: numerical differentiation)
//             computeJacobian(jointAngles, jacobian);
            
//             Matrix j_mat(6, 6);
//             for (int i=0; i<6; i++)
//                 for (int k=0; k<6; k++)
//                    j_mat(i, k) = static_cast<double>(jacobian[i][k]);
//             Matrix inv_j_mat = j_mat.getAdjugate();
//             float jacobianPseudoInv[6][numJoints];
//             /*for (int i=0; i<6; i++)
//                 for (int k=0; k<6; k++)
//                     jacobianPseudoInv[i][k] = inv_j_mat(i, k);
            
//             for(int i = 0; i < 6; i++)  
//             {
//                 for(int j = 0; j < 6; j++)  
//                 {  
//                     Serial.print(jacobianPseudoInv[i][j]);
//                     Serial.print(", ");
//                 }
//                 Serial.println();
//             }*/
//             Serial.println("j_mat");
//             for(int i = 0; i < 6; i++)  
//             {
//                 for(int j = 0; j < 6; j++)  
//                 {  
//                     Serial.print(j_mat(i, j));
//                     Serial.print(", ");
//                 }
//                 Serial.println();
//             }
//             Serial.println("inv_j_mat");
//             for(int i = 0; i < 6; i++)  
//             {
//                 for(int j = 0; j < 6; j++)  
//                 {  
//                     Serial.print(inv_j_mat(i, j));
//                     Serial.print(", ");
//                 }
//                 Serial.println();
//             }

//             // Pseudo-inverse of Jacobian matrix
//             // float jacobianPseudoInv[6][numJoints];
//             // jacobianPseudoInv = inverted(jacobian);
//             // pseudoInverse(jacobian, jacobianPseudoInv);
            
//             // Update joint angles using IK formula: dTheta = J^+ * error
//             float dTheta[numJoints];
//             multiplyMatrixVector(jacobianPseudoInv, error, dTheta);
            
//             // Serial.print("jointAngles:");
//             // for(int i = 0; i < 6; i++)  
//             // {         
//             //     Serial.print(jointAngles[i]);
//             //     Serial.print(", ");
//             // }
//             // Serial.println();
//             // Serial.print("Error:");
//             // for(int i = 0; i < 6; i++)  
//             // {  
//             //     Serial.print(error[i]);
//             //     Serial.print(", ");
//             // }
//             // Serial.println();
//             // Serial.print("jacobian:");
//             // for(int i = 0; i < 6; i++)  
//             // {
//             //     for(int j = 0; j < 6; j++)  
//             //     {  
//             //         Serial.print(jacobian[i][j]);
//             //         Serial.print(", ");
//             //     }
//             //     Serial.println();
//             // }
//             // Serial.println();
//             // Serial.print("jacobianPseudoInv:");
//             // for(int i = 0; i < 6; i++)  
//             // {
//             //     for(int j = 0; j < 6; j++)  
//             //     {  
//             //         Serial.print(jacobianPseudoInv[i][j]);
//             //         Serial.print(", ");
//             //     }
//             //     Serial.println();
//             // }
//             // Serial.println();
//             // Serial.print("dTheta:");
//             // for(int i = 0; i < 6; i++)  
//             // {   
//             //     Serial.print(dTheta[i]);
//             //     Serial.print(", ");
//             // }
//             // Serial.println();

//             // Update joint angles
//             for (int i = 0; i < numJoints; i++) 
//                 jointAngles[i] += dTheta[i]; // Update joint angles
//         }
//         for (int i = 0; i < numJoints; i++) 
//               jointAngles[i] = normalizeAngle(jointAngles[i]);
//         return false; // IK did not converge within max iterations
//     }

//     void get_ee_data(float* jointAngles,  float* eePose)
//     {
//         for (int i=0; i <6; i++)
//         {
//             eePose[i] = ee_pos->eePose[i] ;
//             jointAngles[i] = ee_pos->jointAngles[i] ;
//         }
//     }
// private:
//     static const int numJoints = 6;
//     static const int DOF = 6;
//     DHParameters* jointParams;
//     float* jointMinAngles;
//     float* jointMaxAngles;
//     EEPos* ee_pos;

//     void matrixMultiply(float A[4][4], float B[4][4]) {
//         float tmp[4][4];

//         for (int i = 0; i < 4; i++) 
//             for (int j = 0; j < 4; j++) {
//                 tmp[i][j] = 0.0;
//                 for (int k = 0; k < 4; k++) 
//                     tmp[i][j] += A[i][k] * B[k][j];
//             }
        

//         for (int i = 0; i < 4; i++) 
//             for (int j = 0; j < 4; j++) 
//                 A[i][j] = tmp[i][j];
//     }

//     void computeJacobian(float* jointAngles, float jacobian[DOF][numJoints]) {
//         // Example: Finite difference method for numerical differentiation
//         // const float delta = 1; // Small delta for numerical differentiation

//         float currentPose[DOF];
//         float perturbedPose[DOF];
//         float delta[6] = {0.5, 0.5, 0.5, 1, 1, 1};
//         computeForwardKinematics(jointAngles, currentPose);

//         for (int i = 0; i < numJoints; i++) 
//         {
//             float perturbedAngles[numJoints];
//             std::copy(jointAngles, jointAngles + numJoints, perturbedAngles);
            
//             perturbedAngles[i] += delta[i];

//             computeForwardKinematics(perturbedAngles, perturbedPose);

//             // Finite difference method: J[i][k] = (Fk(q + delta) - Fk(q)) / delta
//             for (int j = 0; j < DOF; j++)
//               jacobian[i][j] = (perturbedPose[j]- currentPose[j]) / delta[i];
//         }
//     }

//     // void pseudoInverse(float matrix[numJoints][DOF], float pseudoInv[DOF][numJoints]) {
//     //     // Example for 3xnumJoints Jacobian matrix
//     //     // Compute transpose of Jacobian matrix
//     //     float transpose[DOF][numJoints];
//     //     // computeTranspose(matrix, transpose);
//     //     for (int i = 0; i < DOF; i++) {
//     //         for (int j = 0; j < numJoints; j++) {
//     //             transpose[i][j] = matrix[j][i];
//     //         }
//     //     }

//     //     // Compute pseudo-inverse (simple case)
//     //     // (A^T * A)^(-1) * A^T
//     //     // Assuming numJoints is 3 for this example
//     //     // Placeholder: Invert 3x3 matrix (replace with actual implementation)
//     //     // Example: Pseudo-inverse computation
//     //     // Assuming numJoints (columns) is 3 for this example
//     //     // Placeholder: Invert 3x3 matrix (replace with actual implementation)
//     //     // Example: Pseudo-inverse computation

//     //     float product[DOF][numJoints];
//     //     // matrixMultiply(transpose, matrix, product);

//     //     for (int i = 0; i < DOF; i++) 
//     //     {
//     //         for (int j = 0; j < numJoints; j++) {
//     //             float sum = 0.0;
//     //             for (int k = 0; k < DOF; k++) 
//     //                 sum += transpose[i][k] * matrix[k][j];
//     //             product[i][j] = sum;
//     //         }
//     //     }

//     //     // Compute pseudo-inverse: (A^T * A)^(-1) * A^T
//     //     // Inverse of product (3x3 matrix)
//     //     float determinant = product[0][0] * product[1][1] * product[2][2] -
//     //                         product[0][0] * product[1][2] * product[2][1] -
//     //                         product[0][1] * product[1][0] * product[2][2] +
//     //                         product[0][1] * product[1][2] * product[2][0] +
//     //                         product[0][2] * product[1][0] * product[2][1] -
//     //                         product[0][2] * product[1][1] * product[2][0];

//     //     // Calculate inverse of determinant
//     //     float invDet = 1.0 / determinant;

//     //     // Calculate the inverse matrix
//     //     pseudoInv[0][0] = (product[1][1] * product[2][2] - product[1][2] * product[2][1]) * invDet;
//     //     pseudoInv[0][1] = (product[0][2] * product[2][1] - product[0][1] * product[2][2]) * invDet;
//     //     pseudoInv[0][2] = (product[0][1] * product[1][2] - product[0][2] * product[1][1]) * invDet;
//     //     pseudoInv[1][0] = (product[1][2] * product[2][0] - product[1][0] * product[2][2]) * invDet;
//     //     pseudoInv[1][1] = (product[0][0] * product[2][2] - product[0][2] * product[2][0]) * invDet;
//     //     pseudoInv[1][2] = (product[0][2] * product[1][0] - product[0][0] * product[1][2]) * invDet;
//     //     pseudoInv[2][0] = (product[1][0] * product[2][1] - product[1][1] * product[2][0]) * invDet;
//     //     pseudoInv[2][1] = (product[0][1] * product[2][0] - product[0][0] * product[2][1]) * invDet;
//     //     pseudoInv[2][2] = (product[0][0] * product[1][1] - product[0][1] * product[1][0]) * invDet;
//     // }
    
//     // Helper function to normalize angle to [-180, 180] degrees
//     float normalizeAngle(float angle) 
//     {
//         if (angle > 10000)  return 200;
//         if (angle < -10000) return -200;
//         while (angle <= -180.0) 
//             angle += 360.0;
//         while (angle > 180.0) 
//             angle -= 360.0;
//         return angle;
//     }
    
//     void multiplyMatrixVector(float matrix[][numJoints], float vector[], float result[]) {
//         // Multiply matrix (numJoints x 6) with vector (6x1) to get result (numJointsx1)
//         for (int i = 0; i < numJoints; i++) {
//             float sum = 0.0;
//             for (int j = 0; j < DOF; j++) 
//                 sum += matrix[i][j] * vector[j];
            
//             result[i] = sum;
//         }
//     }
// };

// void init_arm(RobotArm& arm)
// {
//      // Set DH parameters for each joint
//     // Example DH parameters for a 4-DOF arm (replace with actual values)
//     arm.setDHParameters(0, 0.0, 90.0, 94.85, 0.0);  // Joint 1
//     arm.setDHParameters(1, 104, 0.0, 0.0, -90.0);  // Joint 2
//     arm.setDHParameters(2, 104, 0, 0.0, 180.0);   // Joint 3
//     arm.setDHParameters(3, 0.0, 90.0, -7.65, 0.0);   // Joint 4
//     arm.setDHParameters(4, 0.0, -90.0, 66.6, 0.0);   // Joint 4
//     arm.setDHParameters(5, 0.0, 0.0, 66.6, 0.0);   // Joint 4

//     // Set joint angle limits for each joint
//     arm.setJointAngleLimits(0, -90.0, 90.0);  // Joint 1
//     arm.setJointAngleLimits(1, -90.0, 90.0);  // Joint 2
//     arm.setJointAngleLimits(2, -90.0, 90.0);  // Joint 3
//     arm.setJointAngleLimits(3, -90.0, 90.0);  // Joint 4
//     arm.setJointAngleLimits(4, -90.0, 90.0);  // Joint 5
//     arm.setJointAngleLimits(5, -90.0, 90.0);  // Joint 6
// }
// void fw_example(RobotArm& arm)
// {
//     // Forward Kinematics Examples
//     float jointAngles[6] = {90, 90.0, 0, 90, 0, 0}; // Joint angles in degrees
//     // Compute forward kinematics
//     arm.computeForwardKinematics(jointAngles);
//     float eePose[6]; // End-effector pose (x, y, z, roll, pitch, yaw)
//     arm.get_ee_data(jointAngles, eePose);
//     Serial.println("End-Effector Pose:");
//     Serial.print("Position (XYZ): ");
//     Serial.print(eePose[0]);
//     Serial.print(", ");
//     Serial.print(eePose[1]);
//     Serial.print(", ");
//     Serial.println(eePose[2]);
//     Serial.print("Orientation (RPY): ");
//     Serial.print(eePose[3]);
//     Serial.print(", ");
//     Serial.print(eePose[4]);
//     Serial.print(", ");
//     Serial.println(eePose[5]);
//     //
// }


// // Example usage:
// void ex() {
//     // Create a robotic arm with 4 joints
//     RobotArm arm;
//     init_arm(arm);
//     fw_example(arm);
//     // ik_example
//     float desiredPose[6] = {58.95, 0, 369.56, -90, 0, -90.0}; // Example desired end-effector pose
//     // Compute inverse kinematics to find joint angles for desired end-effector pose
//     float jointAngles[6]; // Array to store joint angles (6-DOF arm)
//     bool success = arm.computeInverseKinematics(desiredPose, jointAngles);
//     Serial.println("Finish IK");

//     if (success) 
//         // Print computed joint angles
//         Serial.println("Computed Joint Angles:");
//     else 
//         Serial.println("Inverse Kinematics failed to converge.");
//     for (int i = 0; i < 6; i++) 
//     {
//         Serial.print(jointAngles[i]);
//         Serial.print(", ");
//     }
//     Serial.println();
    
// }

// void setup() {
//     // put your setup code here, to run once:
//     Serial.begin(115200);
//     ex();
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   delay(10); // this speeds up the simulation
// }

