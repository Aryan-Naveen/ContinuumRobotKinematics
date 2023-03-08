import numpy as np
from scipy.spatial.transform import Rotation as RM

n_segments = 3
segmentLengths = [5, 5, 5]


def forwardKinematics(theta):
    R = np.eye(3)
    t = np.zeros((1, 3))
    for segId in range(n_segments):
        kappa = max(theta[2*segId], 0.0000001)

        phi = theta[2*segId + 1]
        def getRotationBetweenSegments(s_, k_, p_):
            return np.array([[np.cos(p_)**2 * (np.cos(k_*s_) - 1) + 1             , np.sin(p_)*np.cos(p_)*(np.cos(k_*s_) - 1)             , -np.cos(p_)*np.sin(k_*s_)],
                             [np.sin(p_)*np.cos(p_)*(np.cos(k_*s_) - 1)           , np.cos(p_)**2 * (1 - np.cos(k_*s_)) + np.cos(k_*s_)   , -np.sin(p_)*np.sin(k_*s_)],
                             [np.cos(p_)*np.sin(k_*s_)                            , np.sin(p_)*np.sin(k_*s_)                              , np.cos(k_*s_)]])
        def getTranslationVectorBetweenSegments(s_, k_, p_):
            t = np.array([np.cos(p_) * (np.cos(k_ * s_) - 1)/k_, np.sin(p_)*(np.cos(k_*s_) - 1)/k_, np.sin(k_*s_)/k_])
            return R @ t

        R = R @ getRotationBetweenSegments(segmentLengths[segId], kappa, phi)


        t = t + getTranslationVectorBetweenSegments(segmentLengths[segId], kappa, phi)

    print(t)
    print(RM.from_matrix(R).as_euler('xyz'))

forwardKinematics(np.array([0.5, 3.1415, 0.3, 1.5, 0, 0]))
# Vector6d Continuum::forwardKinematics(Vector6d theta){
# 	double kappa, phi;
# 	tfScalar roll, pitch, yaw;
# 	tf::Matrix3x3 finalOrientation = tf::Matrix3x3();
# 	tf::Matrix3x3 Rot;
#
# 	tf::Vector3 finalTrans;
# 	finalTrans.setValue(0, 0, 0);
#
# 	finalOrientation.setIdentity();
# 	for(int segID=0; segID < 3; segID++){
# 			kappa = theta(2*segID);
# 			phi = theta(2*segID + 1);
# 			if(kappa == 0) kappa = 0.0000001;
# 			kappa = std::min(kappa,1/segmentLength[segID]);
#
# 			Rot.setValue(pow(cos(phi),2) * (cos(kappa*segmentLength[segID]) - 1) + 1, sin(phi)*cos(phi)*( cos(kappa*segmentLength[segID]) - 1), -cos(phi)*sin(kappa*segmentLength[segID]),
# 									sin(phi)*cos(phi)*( cos(kappa*segmentLength[segID]) - 1), pow(cos(phi),2) * ( 1 - cos(kappa*segmentLength[segID]) ) + cos( kappa * segmentLength[segID] ),  -sin(phi)*sin(kappa*segmentLength[segID]),
# 									 cos(phi)*sin(kappa*segmentLength[segID]),  sin(phi)*sin(kappa*segmentLength[segID]), cos(kappa*segmentLength[segID]));
# 			finalOrientation *= Rot;
# 			finalTrans += (tf::Matrix3x3(basePose[segID].getRotation())*tf::Vector3(cos(phi)*( cos(kappa*segmentLength[segID]) - 1)/kappa, sin(phi)*( cos(kappa*segmentLength[segID]) - 1)/kappa, sin(kappa*segmentLength[segID])/kappa));
#
# 	}
# 	finalOrientation.getEulerYPR(yaw, pitch, roll);
# 	Vector6d finalPose;
# 	finalPose << finalTrans[0], finalTrans[1], finalTrans[2], (double)roll, (double)pitch, (double)yaw;
# 	return finalPose;
# }
