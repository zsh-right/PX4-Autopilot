#ifndef EKF_EKF_WRAPPER_H
#define EKF_EKF_WRAPPER_H

#include <memory>
#include "EKF/ekf.h"
#include "EKF/estimator_interface.h"

class EkfWrapper
{
public:
    EkfWrapper(std::shared_ptr<Ekf> ekf);
    ~EkfWrapper();

    void setBaroHeightRef();
    void enableBaroHeightFusion();
    void disableBaroHeightFusion();
    bool isIntendingBaroHeightFusion() const;

    void setGpsHeightRef();
    void enableGpsHeightFusion();
    void disableGpsHeightFusion();
    bool isIntendingGpsHeightFusion() const;

    void setRangeHeightRef();
    void enableRangeHeightFusion();
    void disableRangeHeightFusion();
    bool isIntendingRangeHeightFusion() const;

    void setExternalVisionHeightRef();
    void enableExternalVisionHeightFusion();
    /* void disableExternalVisionHeightFusion(); */
    bool isIntendingExternalVisionHeightFusion() const;

    void enableBetaFusion();
    void disableBetaFusion();
    bool isIntendingBetaFusion() const;

    bool isIntendingAirspeedFusion() const;

    void enableGpsFusion();
    void disableGpsFusion();
    bool isIntendingGpsFusion() const;

    void enableGpsHeadingFusion();
    void disableGpsHeadingFusion();
    bool isIntendingGpsHeadingFusion() const;

    void enableFlowFusion();
    void disableFlowFusion();
    bool isIntendingFlowFusion() const;
    void setFlowOffset(const matrix::Vector3f &offset);

    void enableExternalVisionPositionFusion();
    void disableExternalVisionPositionFusion();
    bool isIntendingExternalVisionPositionFusion() const;

    void enableExternalVisionVelocityFusion();
    void disableExternalVisionVelocityFusion();
    bool isIntendingExternalVisionVelocityFusion() const;

    void enableExternalVisionHeadingFusion();
    void disableExternalVisionHeadingFusion();
    bool isIntendingExternalVisionHeadingFusion() const;

    bool isIntendingMagHeadingFusion() const;
    bool isIntendingMag3DFusion() const;
    bool isMagHeadingConsistent() const;
    void setMagFuseTypeNone();
    void enableMagStrengthCheck();
    void enableMagInclinationCheck();
    void enableMagCheckForceWMM();

    bool isWindVelocityEstimated() const;

    bool isIntendingTerrainRngFusion() const;

    bool isIntendingTerrainFlowFusion() const;

    Eulerf getEulerAngles() const;
    float getYawAngle() const;
    int getQuaternionResetCounter() const;

    void enableDragFusion();
    void disableDragFusion();
    void setDragFusionParameters(const float &bcoef_x, const float &bcoef_y, const float &mcoef);

    float getMagHeadingNoise() const;

    void enableGyroBiasEstimation();
    void disableGyroBiasEstimation();

private:
    std::shared_ptr<Ekf> _ekf;

    // Pointer to Ekf internal param struct
    parameters *_ekf_params;

};
#endif // !EKF_EKF_WRAPPER_H
