#ifndef TankDrivePWM_h
#define TankDrivePWM_h

#include "ServoDispatch.h"
#include "drive/TankDrive.h"

/**
  * \ingroup drive
  *
  * \class TankDrivePWM
  *
  * \brief Base template of automatic forwarder from i2c to CommandEvent
  *
  *
  * \code
  * #include "drive/TankDrivePWM.h"
  *
  * TankDrivePWM tankDrive;
  * \endcode
  *
  */
class TankDrivePWM : public TankDrive
{
public:
    /** \brief Constructor
      *
      * Will drive PWM pins
      */
    TankDrivePWM(ServoDispatch& dispatch, uint8_t leftNum, uint8_t rightNum, JoystickController& driveStick) :
        TankDrivePWM(dispatch, leftNum, rightNum, -1, driveStick)
    {
    }

    /** \brief Constructor
      *
      * Will drive PWM pins
      */
    TankDrivePWM(ServoDispatch& dispatch, uint8_t leftNum, uint8_t rightNum, int throttleNum, JoystickController& driveStick) :
        TankDrive(driveStick),
        fDispatch(dispatch),
        fLeft(leftNum),
        fRight(rightNum),
        fThrottle(throttleNum)
    {
        // For PWM drive we want to default to middle position so motors are stopped initially
        for (uint16_t i = 0; i < dispatch.getNumServos(); i++)
        {
            uint16_t middle = (dispatch.getMinimum(i) + dispatch.getMaximum(i)) / 2;
            dispatch.setNeutral(i, middle);
        }

        setMaxSpeed(1.0f);
    }

    virtual void setup() override
    {
    }

    virtual void stop() override
    {
        fDispatch.moveTo(fLeft, 0.5);
        fDispatch.moveTo(fRight, 0.5);
        TankDrive::stop();
    }

protected:
    ServoDispatch& fDispatch;
    uint8_t fLeft;
    uint8_t fRight;
    int fThrottle;

    virtual void motor(float left, float right, float throttle) override
    {
        printf("motor(%f, %f, %f)", left, right, throttle);
        if (fThrottle != -1)
        {
            left = map(left, -1.0f, 1.0f, 0.0f, 1.0f);
            right = map(right, -1.0f, 1.0f, 0.0f, 1.0f);
        }
        else
        {
            // Premultiply throttle if no separate throttle pin
            left = map(left * throttle, -1.0f, 1.0f, 0.0f, 1.0f);
            right = map(right * throttle, -1.0f, 1.0f, 0.0f, 1.0f);
        }

        printf(" -> left(%f), right(%f)\n", left, right);
        if (fThrottle != -1)
        {
            fDispatch.moveTo(fLeft, left);
            fDispatch.moveTo(fRight, right);
            fDispatch.moveTo(fThrottle, throttle);
        }
        else
        {
            fDispatch.moveTo(fLeft, left);
            fDispatch.moveTo(fRight, right);
        }
    }

    virtual bool hasThrottle() override
    {
        return (fThrottle != -1);
    }

    static float map(float x, float in_min, float in_max, float out_min, float out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
};
#endif
