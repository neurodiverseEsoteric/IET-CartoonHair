/*
-----------------------------------------------------------------------------
Filename:    CartoonHairSimulation.h
-----------------------------------------------------------------------------


This source file is generated by the
   ___                   _              __    __ _                  _ 
  /___\__ _ _ __ ___    /_\  _ __  _ __/ / /\ \ (_)______ _ _ __ __| |
 //  // _` | '__/ _ \  //_\\| '_ \| '_ \ \/  \/ / |_  / _` | '__/ _` |
/ \_// (_| | | |  __/ /  _  \ |_) | |_) \  /\  /| |/ / (_| | | | (_| |
\___/ \__, |_|  \___| \_/ \_/ .__/| .__/ \/  \/ |_/___\__,_|_|  \__,_|
      |___/                 |_|   |_|                                 
      Ogre 1.8.x Application Wizard for VC10 (May 2012)
      https://bitbucket.org/jacmoe/ogreappwizards
-----------------------------------------------------------------------------
*/

#include "stdafx.h"
//#include "HairModel.h"
//#include "DebugDrawer.h"
#include "IdBufferRenderTargetListener.h"

#ifndef __CartoonHairSimulation_h_
#define __CartoonHairSimulation_h_

class CartoonHairSimulation : public Ogre::FrameListener, public Ogre::WindowEventListener, public OIS::KeyListener, public OIS::MouseListener, OgreBites::SdkTrayListener
{
public:
    CartoonHairSimulation(void);
    virtual ~CartoonHairSimulation(void);

    void go(void);

protected:
    bool setup();
    bool configure(void);
    void chooseSceneManager(void);
    void createCamera(void);
    void createFrameListener(void);
	void createScene(void);
    void destroyScene(void);
    void createViewports(void);
    void setupResources(void);
    void createResourceListener(void);
    void loadResources(void);

    // Ogre::FrameListener
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

    // OIS::KeyListener
    virtual bool keyPressed( const OIS::KeyEvent &arg );
    virtual bool keyReleased( const OIS::KeyEvent &arg );
    // OIS::MouseListener
    virtual bool mouseMoved( const OIS::MouseEvent &arg );
    virtual bool mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id );
    virtual bool mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id );

    // Ogre::WindowEventListener
    //Adjust mouse clipping area
    virtual void windowResized(Ogre::RenderWindow* rw);
    //Unattach OIS before window shutdown (very important under Linux)
    virtual void windowClosed(Ogre::RenderWindow* rw);

    Ogre::Root *mRoot;
    Ogre::Camera* mCamera;
    Ogre::SceneManager* mSceneMgr;
    Ogre::RenderWindow* mWindow;
    Ogre::String mResourcesCfg;
    Ogre::String mPluginsCfg;

	CEGUI::OgreRenderer* m_renderer;
	CEGUI::Window *m_guiRoot;
	CEGUI::Slider *m_edgeSlider, *m_bendingSlider, *m_torsionSlider, *m_stictionSlider;
	CEGUI::Slider *m_aSlider, *m_bSlider, *m_cSlider;

    // OgreBites
    OgreBites::SdkTrayManager* mTrayMgr;
    OgreBites::SdkCameraMan* mCameraMan;     // basic camera controller
    OgreBites::ParamsPanel* mDetailsPanel;   // sample details panel
    bool mCursorWasVisible;                  // was cursor visible before dialog appeared
    bool mShutDown;

    //OIS Input devices
    OIS::InputManager* mInputManager;
    OIS::Mouse*    mMouse;
    OIS::Keyboard* mKeyboard;

	//Bullet physics
	btSoftRigidDynamicsWorld *mWorld;
	btDispatcher *mDispatcher;
	btCollisionConfiguration *mCollisionConfig;
	btBroadphaseInterface *mBroadphase;
	btSequentialImpulseConstraintSolver *mConstraintSolver;
	btSoftBodySolver *mSoftBodySolver;

	btRigidBody *m_headRigidBody;
	Ogre::SceneNode *m_headNode;
	Ogre::SceneNode *m_characterNode;
	Ogre::Bone *m_headBone;
	Ogre::Entity *m_character;
	Ogre::AnimationState *m_characterAnimationState;

	//spring materials
	btSoftBody::Material *m_edgeMaterial;
	btSoftBody::Material *m_bendingMaterial;
	btSoftBody::Material *m_torsionMaterial;
	btSoftBody::Material *m_stictionMaterial;

	DebugDrawer *m_debugDrawer;

	HairModel *m_hairModel;

	IdBufferRenderTargetListener *m_idBufferListener;

	bool m_cameraControl;
	bool m_physicsEnabled;
};

#endif // #ifndef __CartoonHairSimulation_h_
