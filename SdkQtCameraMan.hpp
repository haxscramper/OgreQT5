#ifndef SDKQTCAMERAMAN_HPP
#define SDKQTCAMERAMAN_HPP
/*
 -----------------------------------------------------------------------------
 This source file is part of OGRE
 (Object-oriented Graphics Rendering Engine)
 For the latest info, see http://www.ogre3d.org/

 Copyright (c) 2000-2014 Torus Knot Software Ltd

 Permission is hereby granted, free of charge, to any person obtaining a
 copy of this software and associated documentation files (the "Software"),
 to deal in the Software without restriction, including without limitation
 the rights to use, copy, modify, merge, publish, distribute, sublicense,
 and/or sell copies of the Software, and to permit persons to whom the
 Software is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 DEALINGS IN THE SOFTWARE.
 -----------------------------------------------------------------------------
 */
// File modified to change OIS to Qt KeyEvents

#include "OgreCamera.h"
#include "OgreFrameListener.h"
#include "OgreSceneNode.h"
#include <QKeyEvent>
#include <QMouseEvent>
#include <OgreSceneManager.h>

// enum CameraStyle should be in other namespace than
// OgreBites::CameraStyle
namespace OgreQtBites {
enum CameraStyle // enumerator values for different styles of camera
                 // movement
{
    CS_FREELOOK,
    CS_ORBIT,
    CS_MANUAL
};

/*=============================================================================
| Utility class for controlling the camera in samples.
=============================================================================*/
class SdkQtCameraMan
{
  public:
    SdkQtCameraMan(Ogre::Camera* cam, Ogre::SceneNode* camNode)
        : mCamera(0), mTarget(0), mVelocity(Ogre::Vector3::ZERO) {

        setCamera(cam);
        mCameraNode = camNode;
        setStyle(CS_FREELOOK);
    }

    virtual ~SdkQtCameraMan() {}

    /*-----------------------------------------------------------------------------
    | Swaps the camera on our camera man for another camera.
    -----------------------------------------------------------------------------*/
    virtual void setCamera(Ogre::Camera* cam) { mCamera = cam; }

    virtual Ogre::Camera* getCamera() { return mCamera; }

    /*-----------------------------------------------------------------------------
    | Sets the target we will revolve around. Only applies for orbit style.
    -----------------------------------------------------------------------------*/
    virtual void setTarget(Ogre::SceneNode* target) {
        if (target != mTarget) {
            mTarget = target;
            if (target) {
                setYawPitchDist(Ogre::Degree(0), Ogre::Degree(15), 150);
                mCameraNode->setAutoTracking(true, mTarget);
            } else {
                mCameraNode->setAutoTracking(false);
            }
        }
    }

    virtual Ogre::SceneNode* getTarget() { return mTarget; }

    /*-----------------------------------------------------------------------------
    | Sets the spatial offset from the target. Only applies for orbit
    style.
    -----------------------------------------------------------------------------*/
    virtual void setYawPitchDist(
        Ogre::Radian yaw,
        Ogre::Radian pitch,
        Ogre::Real   dist) {
        mCameraNode->setPosition(mTarget->_getDerivedPosition());
        mCameraNode->setOrientation(mTarget->_getDerivedOrientation());
        mCameraNode->yaw(yaw);
        mCameraNode->pitch(-pitch);
        mCameraNode->translate(
            Ogre::Vector3(0, 0, dist), Ogre::Node::TS_LOCAL);
    }

    /*-----------------------------------------------------------------------------
    | Sets the camera's top speed. Only applies for free-look style.
    -----------------------------------------------------------------------------*/
    virtual void setTopSpeed(Ogre::Real topSpeed) {
        config.topSpeed = topSpeed;
    }

    virtual Ogre::Real getTopSpeed() { return config.topSpeed; }

    /*-----------------------------------------------------------------------------
    | Sets the movement style of our camera man.
    -----------------------------------------------------------------------------*/
    virtual void setStyle(CameraStyle style) {
        if (mStyle != CS_ORBIT && style == CS_ORBIT) {
            setTarget(
                mTarget ? mTarget
                        : mCamera->getSceneManager()->getRootSceneNode());
            mCameraNode->setFixedYawAxis(true);
            manualStop();
            setYawPitchDist(Ogre::Degree(0), Ogre::Degree(15), 150);
        } else if (mStyle != CS_FREELOOK && style == CS_FREELOOK) {
            mCameraNode->setAutoTracking(false);
            mCameraNode->setFixedYawAxis(true);
        } else if (mStyle != CS_MANUAL && style == CS_MANUAL) {
            mCameraNode->setAutoTracking(false);
            manualStop();
        }
        mStyle = style;
    }

    virtual CameraStyle getStyle() { return mStyle; }

    /*-----------------------------------------------------------------------------
    | Manually stops the camera when in free-look mode.
    -----------------------------------------------------------------------------*/
    virtual void manualStop() {
        if (mStyle == CS_FREELOOK) {
            controlStte.goingForward = false;
            controlStte.goingBack    = false;
            controlStte.goingLeft    = false;
            controlStte.goingRight   = false;
            controlStte.goingUp      = false;
            controlStte.goingDown    = false;
            mVelocity                = Ogre::Vector3::ZERO;
        }
    }

    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt) {
        if (mStyle == CS_FREELOOK) {
            // build our acceleration vector based on keyboard input
            // composite
            Ogre::Vector3 accel = Ogre::Vector3::ZERO;
            if (controlStte.goingForward) {
                // was `getDirection()`
                accel += mCameraNode->getOrientation().zAxis() * -1;
            }
            if (controlStte.goingBack) {
                accel -= mCameraNode->getOrientation().zAxis() * -1;
            }
            if (controlStte.goingRight) {
                accel += mCameraNode->getOrientation().xAxis();
            }
            if (controlStte.goingLeft) {
                accel -= mCameraNode->getOrientation().xAxis();
            }
            if (controlStte.goingUp) {
                accel += mCameraNode->getOrientation().yAxis();
            }
            if (controlStte.goingDown) {
                accel -= mCameraNode->getOrientation().yAxis();
            }

            // if accelerating, try to reach top speed in a certain time
            Ogre::Real topSpeed = controlStte.fastMove
                                      ? config.topFastSpeed
                                      : config.topSpeed;

            if (accel.squaredLength() != 0) {
                accel.normalise();
                mVelocity += accel * topSpeed * evt.timeSinceLastFrame
                             * config.acceleration;
            }
            // if not accelerating, try to stop in a certain time
            else {
                mVelocity -= mVelocity * evt.timeSinceLastFrame
                             * config.deceleration;
            }

            Ogre::Real tooSmall = std::numeric_limits<
                Ogre::Real>::epsilon();

            // keep camera velocity below top speed and above epsilon
            if (mVelocity.squaredLength() > topSpeed * topSpeed) {
                mVelocity.normalise();
                mVelocity *= topSpeed;
            } else if (mVelocity.squaredLength() < tooSmall * tooSmall) {
                mVelocity = Ogre::Vector3::ZERO;
            }

            if (mVelocity != Ogre::Vector3::ZERO) {
                mCameraNode->translate(mVelocity * evt.timeSinceLastFrame);
            }
        }

        return true;
    }

    /*-----------------------------------------------------------------------------
    | Processes key presses for free-look style movement.
    -----------------------------------------------------------------------------*/
    virtual void injectKeyDown(const QKeyEvent& evt) {
        if (mStyle == CS_FREELOOK) {
            if (evt.key() == Qt::Key_W || evt.key() == Qt::Key_Up) {
                controlStte.goingForward = true;
            } else if (
                evt.key() == Qt::Key_S || evt.key() == Qt::Key_Down) {
                controlStte.goingBack = true;
            } else if (
                evt.key() == Qt::Key_A || evt.key() == Qt::Key_Left) {
                controlStte.goingLeft = true;
            } else if (
                evt.key() == Qt::Key_D || evt.key() == Qt::Key_Right) {
                controlStte.goingRight = true;
            } else if (evt.key() == Qt::Key_PageUp) {
                controlStte.goingUp = true;
            } else if (evt.key() == Qt::Key_PageDown) {
                controlStte.goingDown = true;
            } else if (evt.key() == Qt::Key_Shift) {
                controlStte.fastMove = true;
            }
        }
    }

    /*-----------------------------------------------------------------------------
    | Processes key releases for free-look style movement.
    -----------------------------------------------------------------------------*/
    virtual void injectKeyUp(const QKeyEvent& evt) {
        if (mStyle == CS_FREELOOK) {
            if (evt.key() == Qt::Key_W || evt.key() == Qt::Key_Up) {
                controlStte.goingForward = false;
            } else if (
                evt.key() == Qt::Key_S || evt.key() == Qt::Key_Down) {
                controlStte.goingBack = false;
            } else if (
                evt.key() == Qt::Key_A || evt.key() == Qt::Key_Left) {
                controlStte.goingLeft = false;
            } else if (
                evt.key() == Qt::Key_D || evt.key() == Qt::Key_Right) {
                controlStte.goingRight = false;
            } else if (evt.key() == Qt::Key_PageUp) {
                controlStte.goingUp = false;
            } else if (evt.key() == Qt::Key_PageDown) {
                controlStte.goingDown = false;
            } else if (evt.key() == Qt::Key_Shift) {
                controlStte.fastMove = false;
            }
        }
    }

    /*-----------------------------------------------------------------------------
    | Processes mouse movement differently for each style.
    -----------------------------------------------------------------------------*/
    virtual void injectMouseMove(int relX, int relY) {
        if (mStyle == CS_ORBIT) {
            Ogre::Real dist = (mCameraNode->getPosition()
                               - mTarget->_getDerivedPosition())
                                  .length();

            if (controlStte.orbiting) // yaw around the target, and pitch
                                      // locally
            {
                mCameraNode->setPosition(mTarget->_getDerivedPosition());

                mCameraNode->yaw(Ogre::Degree(-relX * 0.025f));
                mCameraNode->pitch(Ogre::Degree(-relY * 0.025f));

                mCameraNode->translate(
                    Ogre::Vector3(0, 0, dist), Ogre::Node::TS_LOCAL);

                // don't let the camera go over the top or around the
                // bottom of the target
            } else if (controlStte.zooming) // move the camera toward or
                                            // away from the target
            {
                // the further the camera is, the faster it moves
                mCameraNode->translate(
                    Ogre::Vector3(0, 0, relY * 0.004f * dist),
                    Ogre::Node::TS_LOCAL);
            }
        } else if (mStyle == CS_FREELOOK) {
            mCameraNode->yaw(Ogre::Degree(-relX * 0.15f));
            mCameraNode->pitch(Ogre::Degree(-relY * 0.15f));
        }
    }

    /*-----------------------------------------------------------------------------
    | Processes mouse movement differently for each style.
    -----------------------------------------------------------------------------*/
    virtual void injectWheelMove(const QWheelEvent& evt) {
        int relZ = evt.delta();
        if (mStyle == CS_ORBIT) {
            Ogre::Real dist = (mCameraNode->getPosition()
                               - mTarget->_getDerivedPosition())
                                  .length();

            if (relZ != 0) // move the camera toward or away from the
                           // target
            {
                // the further the camera is, the faster it moves
                mCameraNode->translate(
                    Ogre::Vector3(0, 0, -relZ * 0.0008f * dist),
                    Ogre::Node::TS_LOCAL);
            }
        }
    }

    /*-----------------------------------------------------------------------------
    | Processes mouse presses. Only applies for orbit style.
    | Left button is for orbiting, and right button is for zooming.
    -----------------------------------------------------------------------------*/
    virtual void injectMouseDown(const QMouseEvent& evt) {
        if (mStyle == CS_ORBIT) {
            if (evt.buttons() & Qt::LeftButton) {
                controlStte.orbiting = true;
            } else if (evt.buttons() & Qt::RightButton) {
                controlStte.zooming = true;
            }
        }
    }

    /*-----------------------------------------------------------------------------
    | Processes mouse releases. Only applies for orbit style.
    | Left button is for orbiting, and right button is for zooming.
    -----------------------------------------------------------------------------*/
    virtual void injectMouseUp(const QMouseEvent& evt) {
        if (mStyle == CS_ORBIT) {
            if (evt.buttons() & Qt::LeftButton) {
                controlStte.orbiting = false;
            } else if (evt.buttons() & Qt::RightButton) {
                controlStte.zooming = false;
            }
        }
    }

  protected:
    Ogre::Camera*    mCamera;
    Ogre::SceneNode* mCameraNode;
    CameraStyle      mStyle;
    Ogre::SceneNode* mTarget;
    Ogre::Vector3    mVelocity;

    struct MoveControlConfig
    {
        int acceleration = 2;
        int deceleration = 10;
        int topSpeed     = 150;
        int topFastSpeed = 150 * 10;
    } config;

    struct MoveControlState
    {
        bool goingForward = false;
        bool goingBack    = false;
        bool goingLeft    = false;
        bool goingRight   = false;
        bool goingUp      = false;
        bool goingDown    = false;
        bool fastMove     = false;
        bool zooming      = false;
        bool orbiting     = false;
    } controlStte;
};
} // namespace OgreQtBites

#endif // SDKQTCAMERAMAN_HPP
