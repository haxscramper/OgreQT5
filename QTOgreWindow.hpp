#ifndef QTOGREWINDOW_HPP
#define QTOGREWINDOW_HPP

/*
 * Qt headers
 */
#include <QtWidgets/QApplication>
#include <QtGui/QKeyEvent>
#include <QtGui/QWindow>

/*
 * Ogre3d header
 */
#include <Ogre.h>

/*
 * Changed SdkCameraMan implementation to work with QKeyEvent, QMouseEvent, QWheelEvent
 */
#include "SdkQtCameraMan.hpp"

class QTOgreWindow : public QWindow, public Ogre::FrameListener
{
    /*
     * A QWindow still inherits from QObject and can have signals/slots; we need to add the appropriate
     * Q_OBJECT keyword so that Qt's intermediate compiler can do the necessary wireup between our class
     * and the rest of Qt.
     */
    Q_OBJECT

public:
    explicit QTOgreWindow(QWindow *parent = NULL);
    ~QTOgreWindow();

    /*
     * We declare these methods virtual to allow for further inheritance.
     */
    virtual void render(QPainter *painter);
    virtual void render();
    virtual void initialize();
    virtual void createScene();
#if OGRE_VERSION >= ((2 << 16) | (0 << 8) | 0)
    virtual void createCompositor();
#endif

    void setAnimating(bool animating);

public slots:

    virtual void renderLater();
    virtual void renderNow();

    /*
     * We use an event filter to be able to capture the keyboard/mouse events. More on this later.
     */
    virtual bool eventFilter(QObject *target, QEvent *event);

signals:
    /*
     * Event for clicking on an entity.
     */
    void entitySelected(Ogre::Entity* entity);

protected:
    /*
     * Ogre4D pointers added here. Useful to have th epointers here for use by the window later.
     */
    Ogre::Root* ogreRoot;
    Ogre::RenderWindow* ogreWindow;
    Ogre::SceneManager* ogreSceneManager;
    Ogre::Camera* ogreCamera;
    Ogre::ColourValue ogreBackground;
    OgreQtBites::SdkQtCameraMan* cameraManager;

    bool updatePending;
    bool animating;

    /*
     * The below methods are what is actually fired when the keys on the keyboard are hit.
     * Similar events are fired when the mouse is pressed or other events occur.
     */
    virtual void keyPressEvent(QKeyEvent* ev);
    virtual void keyReleaseEvent(QKeyEvent* ev);
    virtual void mouseMoveEvent(QMouseEvent* e);
    virtual void wheelEvent(QWheelEvent *e);
    virtual void mousePressEvent(QMouseEvent* e);
    virtual void mouseReleaseEvent(QMouseEvent* e);
    virtual void exposeEvent(QExposeEvent* event);
    virtual bool event(QEvent* event);

    /*
     * FrameListener method
     */
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

    /*
     * Write log messages to Ogre log
     */
    void log(Ogre::String msg);
    void log(QString msg);
};

#endif // QTOGREWINDOW_HPP
