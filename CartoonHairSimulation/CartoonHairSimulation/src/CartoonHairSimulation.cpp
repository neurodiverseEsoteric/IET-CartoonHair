/*
-----------------------------------------------------------------------------
Filename:    CartoonHairSimulation.cpp
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
#include "tinyxml2.h"
#include "CartoonHairSimulation.h"
#include "Constants.h"

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#include "../res/resource.h"
#endif

/*
Convenience functions for getting bone orientation and positions - needed for parenting the hair to the head bone
*/
Ogre::Vector3 localToWorldPosition(Ogre::Bone* bone, Ogre::Entity* entity)
{
	Ogre::Vector3 pos = bone->_getDerivedPosition();
	return entity->getParentSceneNode()->convertLocalToWorldPosition(pos);
}
Ogre::Quaternion localToWorldOrientation(Ogre::Bone* bone, Ogre::Entity* entity)
{
	Ogre::Quaternion orient = bone->_getDerivedOrientation();
	return entity->getParentSceneNode()->convertLocalToWorldOrientation(orient);
}

std::string numberToString(long double value)
{
	try
	{
		return std::to_string(value);
	}
	catch(std::invalid_argument)
	{
		return "0";
	}
}

float stringToFloat(std::string value)
{
	try
	{
		return std::stof(value);
	}
	catch(std::invalid_argument)
	{
		return 0.0f;
	}
}

/*
Convenience function from http://www.ogre3d.org/tikiwiki/tiki-index.php?page=Basic+Tutorial+7
Convertes mouse button presses to a format compatible with CEGUI
*/
CEGUI::MouseButton convertButton(OIS::MouseButtonID buttonID)
{
    switch (buttonID)
    {
    case OIS::MB_Left:
        return CEGUI::LeftButton;
 
    case OIS::MB_Right:
        return CEGUI::RightButton;
 
    case OIS::MB_Middle:
        return CEGUI::MiddleButton;
 
    default:
        return CEGUI::LeftButton;
    }
}

btMultiSphereShape* CartoonHairSimulation::loadHeadShape(std::string filename)
{
	tinyxml2::XMLDocument doc;
	doc.LoadFile(filename.c_str());

	btAlignedObjectArray<btVector3> positions;
	btAlignedObjectArray<btScalar> radii;

	tinyxml2::XMLElement *shape = doc.FirstChildElement()->FirstChildElement();

	for(shape ; shape ; shape = shape->NextSiblingElement())
	{
		btVector3 pos(shape->FloatAttribute("x"),
			shape->FloatAttribute("y"),
			shape->FloatAttribute("z"));
		btScalar rad = shape->FloatAttribute("radius");

		positions.push_back(pos);
		radii.push_back(rad);
	}

	btMultiSphereShape *multiSphere = new btMultiSphereShape(&(positions[0]),&(radii[0]),positions.size());

	positions.clear();
	radii.clear();

	return multiSphere;
}

CartoonHairSimulation::CartoonHairSimulation(void)
    : mRoot(0),
    mCamera(0),
    mSceneMgr(0),
    mWindow(0),
    mResourcesCfg(Ogre::StringUtil::BLANK),
    mPluginsCfg(Ogre::StringUtil::BLANK),
    mTrayMgr(0),
    mCameraMan(0),
    mDetailsPanel(0),
    mCursorWasVisible(false),
    mShutDown(false),
    mInputManager(0),
    mMouse(0),
    mKeyboard(0),
	m_cameraControl(true),
	m_firstTransformation(true),
	m_maxSubSteps(MAX_SUB_STEPS),
	m_imageSpaceSilhouetteEnabled(false)
{
	m_fixedTimeStep= FIXED_TIMESTEP;
	mWorld = NULL;
	mSoftBodySolver = NULL;
	mConstraintSolver = NULL;
	mBroadphase = NULL;
	mDispatcher = NULL;
	mCollisionConfig = NULL;
	m_hairModel = NULL;
	m_renderer = NULL;
	m_headRigidBody = NULL;
	m_edgeMaterial = NULL;
	m_bendingMaterial = NULL;
	m_torsionMaterial = NULL;
	m_anchorMaterial = NULL;
	m_idBufferListener = NULL;
	m_headBone = NULL;
	m_skeletonDrawer = NULL;
	m_characterAnimationState = NULL;
	m_hairMaterialListener = NULL;
}

//-------------------------------------------------------------------------------------
CartoonHairSimulation::~CartoonHairSimulation(void)
{
	if(m_renderer)
	{
		CEGUI::OgreRenderer::destroySystem();
	}

	if(m_skeletonDrawer)
	{
		delete m_skeletonDrawer;
	}

    if (mTrayMgr) delete mTrayMgr;
    if (mCameraMan) delete mCameraMan;


    //Remove ourself as a Window listener
    Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
    windowClosed(mWindow);

	delete mRoot;

	if(m_hairModel)
	{
		delete m_hairModel;
	}

	if(m_idBufferListener)
	{
		delete m_idBufferListener;
	}

	if(m_hairMaterialListener)
	{
		delete m_hairMaterialListener;
	}

	//clean up physics
	if(mWorld)
	{
		if(m_headRigidBody)
		{
			mWorld->removeRigidBody(m_headRigidBody);
			delete m_headRigidBody->getCollisionShape();
			delete m_headRigidBody->getMotionState();
			delete m_headRigidBody;
		}

		if(m_edgeMaterial)
		{
			delete m_edgeMaterial;
			delete m_bendingMaterial;
			delete m_torsionMaterial;
			delete m_anchorMaterial;
		}

		delete mWorld;
		delete mSoftBodySolver;
		delete mConstraintSolver;
		delete mBroadphase;
		delete mDispatcher;
		delete mCollisionConfig;
	}

	//TO DO: add any clean up you need here
}

//-------------------------------------------------------------------------------------
bool CartoonHairSimulation::configure(void)
{
    // Show the configuration dialog and initialise the system
    // You can skip this and use root.restoreConfig() to load configuration
    // settings if you were sure there are valid ones saved in ogre.cfg
    if(mRoot->showConfigDialog())
    {
        // If returned true, user clicked OK so initialise
        // Here we choose to let the system create a default rendering window by passing 'true'
        mWindow = mRoot->initialise(true, "CartoonHairSimulation Render Window");

        // Let's add a nice window icon
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
        HWND hwnd;
        mWindow->getCustomAttribute("WINDOW", (void*)&hwnd);
        LONG iconID   = (LONG)LoadIcon( GetModuleHandle(0), MAKEINTRESOURCE(IDI_APPICON) );
        SetClassLong( hwnd, GCL_HICON, iconID );
#endif
        return true;
    }
    else
    {
        return false;
    }
}
//-------------------------------------------------------------------------------------
void CartoonHairSimulation::chooseSceneManager(void)
{
    // Get the SceneManager, in this case a generic one
    mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);
}
//-------------------------------------------------------------------------------------
void CartoonHairSimulation::createCamera(void)
{
    // Create the camera
    mCamera = mSceneMgr->createCamera("PlayerCam");

    // Position it at 50 in Z direction
    mCamera->setPosition(Ogre::Vector3(0,0,50));
    // Look back along -Z
    mCamera->lookAt(Ogre::Vector3(0,0,-1));
	mCamera->setNearClipDistance(0.1);
	mCamera->setFarClipDistance(1000);

    mCameraMan = new OgreBites::SdkCameraMan(mCamera);   // create a default camera controller
	mCameraMan->setTopSpeed(4.0f);
}
//-------------------------------------------------------------------------------------
void CartoonHairSimulation::createFrameListener(void)
{
    Ogre::LogManager::getSingletonPtr()->logMessage("*** Initializing OIS ***");
    OIS::ParamList pl;
    size_t windowHnd = 0;
    std::ostringstream windowHndStr;

    mWindow->getCustomAttribute("WINDOW", &windowHnd);
    windowHndStr << windowHnd;
    pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));

    mInputManager = OIS::InputManager::createInputSystem( pl );

    mKeyboard = static_cast<OIS::Keyboard*>(mInputManager->createInputObject( OIS::OISKeyboard, true ));
    mMouse = static_cast<OIS::Mouse*>(mInputManager->createInputObject( OIS::OISMouse, true ));

    mMouse->setEventCallback(this);
    mKeyboard->setEventCallback(this);

    //Set initial mouse clipping size
    windowResized(mWindow);

    //Register as a Window listener
    Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);

    mTrayMgr = new OgreBites::SdkTrayManager("InterfaceName", mWindow, mMouse, this);
    mTrayMgr->showFrameStats(OgreBites::TL_BOTTOMLEFT);
    mTrayMgr->hideCursor();
	mTrayMgr->hideTrays();

    // create a params panel for displaying sample details
    Ogre::StringVector items;
    items.push_back("cam.pX");
    items.push_back("cam.pY");
    items.push_back("cam.pZ");
    items.push_back("");
    items.push_back("cam.oW");
    items.push_back("cam.oX");
    items.push_back("cam.oY");
    items.push_back("cam.oZ");
    items.push_back("");
    items.push_back("Filtering");
    items.push_back("Poly Mode");

    mDetailsPanel = mTrayMgr->createParamsPanel(OgreBites::TL_NONE, "DetailsPanel", 200, items);
    mDetailsPanel->setParamValue(9, "Bilinear");
    mDetailsPanel->setParamValue(10, "Solid");
    mDetailsPanel->hide();

    mRoot->addFrameListener(this);
}
//-------------------------------------------------------------------------------------
void CartoonHairSimulation::destroyScene(void)
{
}
//-------------------------------------------------------------------------------------
void CartoonHairSimulation::createViewports(void)
{
    // Create one viewport, entire window
    Ogre::Viewport* vp = mWindow->addViewport(mCamera);
    vp->setBackgroundColour(Ogre::ColourValue(0.39,0.58,0.92,0.0f));//Ogre::ColourValue(0,0,0));

    // Alter the camera aspect ratio to match the viewport
    mCamera->setAspectRatio(
        Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
}
//-------------------------------------------------------------------------------------
void CartoonHairSimulation::setupResources(void)
{
    // Load resource paths from config file
    Ogre::ConfigFile cf;
    cf.load(mResourcesCfg);

    // Go through all sections & settings in the file
    Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

    Ogre::String secName, typeName, archName;
    while (seci.hasMoreElements())
    {
        secName = seci.peekNextKey();
        Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
        Ogre::ConfigFile::SettingsMultiMap::iterator i;
        for (i = settings->begin(); i != settings->end(); ++i)
        {
            typeName = i->first;
            archName = i->second;
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
                archName, typeName, secName);
        }
    }
}
//-------------------------------------------------------------------------------------
void CartoonHairSimulation::createResourceListener(void)
{
	
}
//-------------------------------------------------------------------------------------
void CartoonHairSimulation::loadResources(void)
{
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}
//-------------------------------------------------------------------------------------
void CartoonHairSimulation::go(void)
{
#ifdef _DEBUG
    mResourcesCfg = "resources_d.cfg";
    mPluginsCfg = "plugins_d.cfg";
#else
    mResourcesCfg = "resources.cfg";
    mPluginsCfg = "plugins.cfg";
#endif

    if (!setup())
        return;

    mRoot->startRendering();

    // clean up
    destroyScene();
}
//-------------------------------------------------------------------------------------
bool CartoonHairSimulation::setup(void)
{
	//setup physics
	mCollisionConfig = new btSoftBodyRigidBodyCollisionConfiguration();
	mDispatcher = new btCollisionDispatcher(mCollisionConfig);
	mBroadphase = new btDbvtBroadphase();
	mConstraintSolver = new btSequentialImpulseConstraintSolver();
	mSoftBodySolver = new btDefaultSoftBodySolver();
	mWorld = new btSoftRigidDynamicsWorld(mDispatcher,mBroadphase,mConstraintSolver,mCollisionConfig,mSoftBodySolver);
	mWorld->getPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());

    mRoot = new Ogre::Root(mPluginsCfg);

    setupResources();

    bool carryOn = configure();
    if (!carryOn) return false;

    chooseSceneManager();
    createCamera();
    createViewports();

    // Set default mipmap level (NB some APIs ignore this)
    Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);

    // Create any resource listeners (for loading screens)
    createResourceListener();
    // Load resources
    loadResources();

	// Create the scene
    createScene();

	createFrameListener();

    return true;
};
//-------------------------------------------------------------------------------------
void CartoonHairSimulation::createScene(void)
{
	// Create a light
	Ogre::Light* light = mSceneMgr->createLight("MainLight");
	light->setPosition(-19,-2.4,-12);

	//setup debug drawer
	m_debugDrawer = new DebugDrawer(mSceneMgr);
	mWorld->setDebugDrawer(m_debugDrawer);

	//setup the gui
	//http://www.ogre3d.org/tikiwiki/tiki-index.php?page=Basic+Tutorial+7
	m_renderer = &CEGUI::OgreRenderer::bootstrapSystem();
	CEGUI::Imageset::setDefaultResourceGroup("Imagesets");
	CEGUI::Font::setDefaultResourceGroup("Fonts");
	CEGUI::Scheme::setDefaultResourceGroup("Schemes");
	CEGUI::WidgetLookManager::setDefaultResourceGroup("LookNFeel");
	CEGUI::WindowManager::setDefaultResourceGroup("Layouts");

	CEGUI::SchemeManager::getSingleton().create("VanillaSkin.scheme");

	m_guiRoot = CEGUI::WindowManager::getSingleton().loadWindowLayout("cartoonhair.layout"); 
	CEGUI::System::getSingleton().setGUISheet(m_guiRoot);
	m_guiRoot->setVisible(false);
	CEGUI::System::getSingleton().setDefaultMouseCursor("Vanilla-Images","MouseArrow");

	//link gui elements
	m_blinnSpecularBox = (CEGUI::Checkbox*) m_guiRoot->getChildRecursive("Root//blinnSpecular");
	m_specularTextureBox = (CEGUI::Checkbox*) m_guiRoot->getChildRecursive("Root//specularTexture");
	m_backlightingTextureBox = (CEGUI::Checkbox*) m_guiRoot->getChildRecursive("Root//backlighting");
	m_depthDetailBox = (CEGUI::Checkbox*) m_guiRoot->getChildRecursive("Root//depthAxis");

	m_animateHairBox = (CEGUI::Checkbox*) m_guiRoot->getChildRecursive("Root//anchorAnimate");
	m_animateSkeletonBox = (CEGUI::Checkbox*) m_guiRoot->getChildRecursive("Root//skeletonAnimate");
	m_fadeSilhouetteBox = (CEGUI::Checkbox*) m_guiRoot->getChildRecursive("Root//fadeSilhouette");
	m_sobelSilhouetteBox = (CEGUI::Checkbox*) m_guiRoot->getChildRecursive("Root//sobelSilhouette");
	m_hatchingBox = (CEGUI::Checkbox*) m_guiRoot->getChildRecursive("Root//hatching");
	m_simpleHatchingBox = (CEGUI::Checkbox*) m_guiRoot->getChildRecursive("Root//simpleHatching");

	m_zMinBox = (CEGUI::MultiLineEditbox*) m_guiRoot->getChildRecursive("Root//zMin");
	m_zScaleBox = (CEGUI::MultiLineEditbox*) m_guiRoot->getChildRecursive("Root//zScale");
	m_blinnSBox = (CEGUI::MultiLineEditbox*) m_guiRoot->getChildRecursive("Root//blinnS");
	m_specTexSBox = (CEGUI::MultiLineEditbox*) m_guiRoot->getChildRecursive("Root//specTexS");
	m_backlightSBox = (CEGUI::MultiLineEditbox*) m_guiRoot->getChildRecursive("Root//backlightS");
	m_strokeScaleBox = (CEGUI::MultiLineEditbox*) m_guiRoot->getChildRecursive("Root//strokeScale");

	m_redBox = (CEGUI::MultiLineEditbox*) m_guiRoot->getChildRecursive("Root/Hair/red");
	m_greenBox = (CEGUI::MultiLineEditbox*) m_guiRoot->getChildRecursive("Root/Hair/green");
	m_blueBox = (CEGUI::MultiLineEditbox*) m_guiRoot->getChildRecursive("Root/Hair/blue");

	m_normalsBox = (CEGUI::Checkbox*) m_guiRoot->getChildRecursive("Root/Debug/normals");
	m_debugEdgesBox = (CEGUI::Checkbox*) m_guiRoot->getChildRecursive("Root/Debug/debugEdges");
	m_showPhysicsBox = (CEGUI::Checkbox*) m_guiRoot->getChildRecursive("Root/Debug/physics");
	m_disablePhysicsBox = (CEGUI::Checkbox*) m_guiRoot->getChildRecursive("Root/Debug/disableSim");
	m_showIdBufferBox = (CEGUI::Checkbox*) m_guiRoot->getChildRecursive("Root/Debug/idBuffer");
	m_bonesBox = (CEGUI::Checkbox*) m_guiRoot->getChildRecursive("Root/Debug/bone");
	
	//set values
	m_zMinBox->setText(numberToString(ZMIN));
	m_zScaleBox->setText(numberToString(R));
	m_blinnSBox->setText(numberToString(BLINN_S));
	m_specTexSBox->setText(numberToString(SPECULAR_TEXTURE_S));
	m_backlightSBox->setText(numberToString(BACKLIGHTING_TEXTURE_S));
	m_strokeScaleBox->setText(numberToString(HATCHING_STROKE_SCALE));
	m_redBox->setText(numberToString(RED));
	m_greenBox->setText(numberToString(GREEN));
	m_blueBox->setText(numberToString(BLUE));

	//setup spring materials
	m_edgeMaterial = new btSoftBody::Material();
	m_bendingMaterial = new btSoftBody::Material();
	m_torsionMaterial = new btSoftBody::Material();
	m_anchorMaterial = new btSoftBody::Material();

	m_edgeMaterial->m_kLST = EDGE_STIFFNESS;
	m_bendingMaterial->m_kLST = BENDING_STIFFNESS;
	m_torsionMaterial->m_kLST = TORSION_STIFFNESS;
	m_anchorMaterial->m_kLST = ANCHOR_STIFFNESS;

	m_characterNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	//animation code from http://www.ogre3d.org/tikiwiki/tiki-index.php?page=Intermediate+Tutorial+1#Setting_up_the_Scene
	m_character = mSceneMgr->createEntity("Character","ninja.mesh");
	m_character->setMaterialName("BaseWhiteNoLighting");
	m_characterNode->setScale(0.4,0.4,0.4);
	m_characterNode->yaw(Ogre::Radian(Ogre::Degree(180)));
	m_characterNode->setPosition(0,-40,0);
	m_characterAnimationState = m_character->getAnimationState("Idle2");
	m_characterAnimationState->setLoop(true);
	m_characterAnimationState->setEnabled(true);

	m_characterNode->attachObject(m_character);

	m_skeletonDrawer = new SkeletonDebug(m_character,mSceneMgr,mCamera);

	m_headNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();

	//setup dynamic hair
	Ogre::SkeletonInstance *skeleton = m_character->getSkeleton();

	m_hairModel = new HairModel("../Hair/",
		"hairanimation.xml", mCamera,light,mWindow,mSceneMgr,m_edgeMaterial,m_torsionMaterial,m_bendingMaterial,m_anchorMaterial,mWorld,
		HAIR_QUADRATIC_A,HAIR_QUADRATIC_B,HAIR_QUADRATIC_C);

	if(skeleton->hasBone("Joint8"))
	{
		m_headBone = skeleton->getBone("Joint8");
	}

	m_idBufferListener = new IdBufferRenderTargetListener(mSceneMgr);

	m_hairModel->getIdBufferTexture()->addListener(m_idBufferListener);

	m_idBufferListener->addObjectToID(m_hairModel->getHairManualObject(),"IETCartoonHair/SolidMaterial");
#ifndef OUTER_SILHOUETTE
	m_idBufferListener->addObjectToIgnore(m_hairModel->getEdgeManualObject());
#else
	m_idBufferListener->addObjectToID(m_hairModel->getEdgeManualObject(),"IETCartoonHair/SolidSilhouetteMaterial");
#endif
	m_idBufferListener->addObjectToIgnore(m_hairModel->getNormalsManualObject());
	m_idBufferListener->addObjectToIgnore(m_hairModel->getDebugEdgesManualObject());
	m_idBufferListener->addObjectToIgnore(m_debugDrawer->getLinesManualObject());
	m_idBufferListener->addObjectToDarken(m_character);



#ifdef STYLISED_SPECULAR
	m_idBufferListener->addObjectToIgnore(m_hairModel->getHighlightManualObject());
#endif

	//if reduce to the correct size in the simulation - the collision becomes inaccurate - instead scaling the simulation
	//http://www.bulletphysics.org/mediawiki-1.5.8/index.php?title=Scaling_The_World
	mWorld->setGravity(mWorld->getGravity()*m_hairModel->getSimulationScale());

	//create head rigidbody
	//btBoxShape *headShape = new btBoxShape(btVector3(2.4,5.0,3.0));
	btMultiSphereShape *headShape = loadHeadShape("../Hair/headshape.xml");
	btDefaultMotionState *headMotionState = new btDefaultMotionState(
		btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));
	btRigidBody::btRigidBodyConstructionInfo headConstructionInfo(0,headMotionState,headShape,btVector3(0,0,0));

	m_headRigidBody = new btRigidBody(headConstructionInfo);
	mWorld->addRigidBody(m_headRigidBody,BODY_GROUP, BODY_GROUP | HAIR_GROUP);

	m_headNode->createChildSceneNode("debuglines")->attachObject(m_debugDrawer->getLinesManualObject());
	m_headNode->createChildSceneNode("normals")->attachObject(m_hairModel->getNormalsManualObject());
	m_headNode->createChildSceneNode("debugedges")->attachObject(m_hairModel->getDebugEdgesManualObject());

	m_headNode->createChildSceneNode("hair")->attachObject(m_hairModel->getHairManualObject());
	m_headNode->createChildSceneNode("silhouettes")->attachObject(m_hairModel->getEdgeManualObject());
#ifdef STYLISED_SPECULAR
	m_headNode->createChildSceneNode("specular")->attachObject(m_hairModel->getHighlightManualObject());
#endif

	m_character->setRenderQueueGroupAndPriority(Ogre::RENDER_QUEUE_1,1);
	m_hairModel->getHairManualObject()->setRenderQueueGroupAndPriority(Ogre::RENDER_QUEUE_2,2);
	m_hairModel->getEdgeManualObject()->setRenderQueueGroupAndPriority(Ogre::RENDER_QUEUE_3,3);
#ifdef STYLISED_SPECULAR
	m_hairModel->getHighlightManualObject()->setRenderQueueGroupAndPriority(Ogre::RENDER_QUEUE_4,4);
#endif

	//setup compositor
	m_hairMaterialListener = new HairMaterialListener();
	Ogre::CompositorManager::getSingleton().addCompositor(mWindow->getViewport(0),"IETCartoonHair/SilhouetteCompositor");
	Ogre::MaterialManager::getSingleton().addListener(m_hairMaterialListener);
}
//-------------------------------------------------------------------------------------
bool CartoonHairSimulation::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    if(mWindow->isClosed())
        return false;

    if(mShutDown)
        return false;

    //Need to capture/update each device
    mKeyboard->capture();
    mMouse->capture();

	CEGUI::System::getSingleton().injectTimePulse(evt.timeSinceLastFrame);

    mTrayMgr->frameRenderingQueued(evt);

    if (!mTrayMgr->isDialogVisible())
    {
        mCameraMan->frameRenderingQueued(evt);   // if dialog isn't up, then update the camera
        if (mDetailsPanel->isVisible())   // if details panel is visible, then update its contents
        {
            mDetailsPanel->setParamValue(0, Ogre::StringConverter::toString(mCamera->getDerivedPosition().x));
            mDetailsPanel->setParamValue(1, Ogre::StringConverter::toString(mCamera->getDerivedPosition().y));
            mDetailsPanel->setParamValue(2, Ogre::StringConverter::toString(mCamera->getDerivedPosition().z));
            mDetailsPanel->setParamValue(4, Ogre::StringConverter::toString(mCamera->getDerivedOrientation().w));
            mDetailsPanel->setParamValue(5, Ogre::StringConverter::toString(mCamera->getDerivedOrientation().x));
            mDetailsPanel->setParamValue(6, Ogre::StringConverter::toString(mCamera->getDerivedOrientation().y));
            mDetailsPanel->setParamValue(7, Ogre::StringConverter::toString(mCamera->getDerivedOrientation().z));
        }
    }

	float timestep = evt.timeSinceLastFrame;

	if(m_bonesBox->isSelected())
	{
		m_skeletonDrawer->update();
	};

	m_skeletonDrawer->showBones(m_bonesBox->isSelected());
	m_skeletonDrawer->showNames(m_bonesBox->isSelected());
	m_skeletonDrawer->showAxes(m_bonesBox->isSelected());

	//toggle silhouette modes
	m_hairModel->enableSobel(m_sobelSilhouetteBox->isSelected());
	if(m_sobelSilhouetteBox->isSelected())
	{
		Ogre::CompositorManager::getSingleton().setCompositorEnabled(mWindow->getViewport(0),"IETCartoonHair/SilhouetteCompositor",true);
		m_hairModel->getEdgeManualObject()->setVisible(false);
	}
	else
	{
		Ogre::CompositorManager::getSingleton().setCompositorEnabled(mWindow->getViewport(0),"IETCartoonHair/SilhouetteCompositor",false);
		m_hairModel->getEdgeManualObject()->setVisible(true);
	}

	//toggle debug visibility
	m_hairModel->getDebugEdgesManualObject()->setVisible(m_debugEdgesBox->isSelected());
	m_hairModel->getNormalsManualObject()->setVisible(m_normalsBox->isSelected());
	m_debugDrawer->getLinesManualObject()->setVisible(m_showPhysicsBox->isSelected());

	m_hairModel->enableSimpleHatching(m_simpleHatchingBox->isSelected());
	m_hairModel->enableBlinnSpecular(m_blinnSpecularBox->isSelected());
	m_hairModel->enableSpecularTexture(m_specularTextureBox->isSelected());
	m_hairModel->enableBacklightingTexture(m_backlightingTextureBox->isSelected());
	m_hairModel->enableDepthDetailAxis(m_depthDetailBox->isSelected());
	m_hairModel->enableVariableSilhouetteIntensity(m_fadeSilhouetteBox->isSelected());
	m_hairModel->enableHatching(m_hatchingBox->isSelected());

	m_hairModel->setZMin(stringToFloat(m_zMinBox->getText().c_str()));
	m_hairModel->setZScale(stringToFloat(m_zScaleBox->getText().c_str()));
	m_hairModel->setBlinnS(stringToFloat(m_blinnSBox->getText().c_str()));
	m_hairModel->setSpecularTextureS(stringToFloat(m_specTexSBox->getText().c_str()));
	m_hairModel->setBacklightingS(stringToFloat(m_backlightSBox->getText().c_str()));
	m_hairModel->setStrokeScale(stringToFloat(m_strokeScaleBox->getText().c_str()));

	m_hairModel->setHairColour(Ogre::Vector4(
		stringToFloat(m_redBox->getText().c_str()),
		stringToFloat(m_greenBox->getText().c_str()),
		stringToFloat(m_blueBox->getText().c_str()),
		1.0f
		));

	m_idBufferListener->setVisible(m_showIdBufferBox->isSelected());

	//physics update - note:  must (timeStep < maxSubSteps*fixedTimeStep) == true according to http://bulletphysics.org/mediawiki-1.5.8/index.php/Stepping_The_World
	//if odd physics problems arise - consider adding parameters for maxSubstep and fixedTimeStep

	if(!m_disablePhysicsBox->isSelected())
	{
		if(m_animateSkeletonBox->isSelected() && m_characterAnimationState)
		{
			m_characterAnimationState->addTime(timestep*ANIMATION_SPEED);
		}
		if(m_headBone)
		{
			//based on code from http://linode.ogre3d.org/forums/viewtopic.php?f=2&t=29717
			Ogre::Vector3 bonePosition = localToWorldPosition(m_headBone,m_character);
			Ogre::Quaternion boneOrientation = localToWorldOrientation(m_headBone,m_character);
			boneOrientation = INITIAL_ORIENTATION*boneOrientation;
			m_hairModel->applyHeadTransform(m_firstTransformation,bonePosition,boneOrientation);
			btVector3 bBonePosition(bonePosition.x,bonePosition.y,bonePosition.z);
			btQuaternion bBoneOrientation(boneOrientation.x,boneOrientation.y,boneOrientation.z,boneOrientation.w);

			m_headRigidBody->setWorldTransform(btTransform(bBoneOrientation,bBonePosition));
			m_hairModel->updateAnchors(timestep);
			m_firstTransformation = false;
		}
		if(m_animateHairBox->isSelected())
		{
			m_hairModel->updateAnchors(timestep);
		}

		//ensure we aren't losing time http://bulletphysics.org/mediawiki-1.5.8/index.php/Stepping_The_World
		if(timestep > m_maxSubSteps*m_fixedTimeStep)
		{
			//increase substeps
			m_maxSubSteps++;
		}

		mWorld->stepSimulation(timestep,m_maxSubSteps,m_fixedTimeStep);
	}

	m_hairModel->updateManualObject();

	if(m_debugDrawer->getLinesManualObject()->isVisible())
	{
		m_debugDrawer->begin();
		mWorld->debugDrawWorld();
		m_debugDrawer->end();
	}

	//TO DO: add any methods you would like to be called

    return true;
}

//-------------------------------------------------------------------------------------

bool CartoonHairSimulation::keyPressed( const OIS::KeyEvent &arg )
{

    if (mTrayMgr->isDialogVisible()) return true;   // don't process any more keys if dialog is up

    if (arg.key == OIS::KC_F)   // toggle visibility of advanced frame stats
    {
        mTrayMgr->toggleAdvancedFrameStats();
    }
    else if (arg.key == OIS::KC_G)   // toggle visibility of even rarer debugging details
    {
        if (mDetailsPanel->getTrayLocation() == OgreBites::TL_NONE)
        {
            mTrayMgr->moveWidgetToTray(mDetailsPanel, OgreBites::TL_TOPLEFT, 0);
            mDetailsPanel->show();
        }
        else
        {
            mTrayMgr->removeWidgetFromTray(mDetailsPanel);
            mDetailsPanel->hide();
        }
    }
	else if (arg.key == OIS::KC_H)
	{
		if(m_hairModel->getHairManualObject()->isVisible())
		{
			m_hairModel->getHairManualObject()->setVisible(false);
		}
		else
		{
			m_hairModel->getHairManualObject()->setVisible(true);
		}
	}
	else if (arg.key == OIS::KC_E)
	{
		if(m_hairModel->getEdgeManualObject()->isVisible())
		{
			m_hairModel->getEdgeManualObject()->setVisible(false);
		}
		else
		{
			m_hairModel->getEdgeManualObject()->setVisible(true);
		}
	}
    else if (arg.key == OIS::KC_T)   // cycle polygon rendering mode
    {
        Ogre::String newVal;
        Ogre::TextureFilterOptions tfo;
        unsigned int aniso;

        switch (mDetailsPanel->getParamValue(9).asUTF8()[0])
        {
        case 'B':
            newVal = "Trilinear";
            tfo = Ogre::TFO_TRILINEAR;
            aniso = 1;
            break;
        case 'T':
            newVal = "Anisotropic";
            tfo = Ogre::TFO_ANISOTROPIC;
            aniso = 8;
            break;
        case 'A':
            newVal = "None";
            tfo = Ogre::TFO_NONE;
            aniso = 1;
            break;
        default:
            newVal = "Bilinear";
            tfo = Ogre::TFO_BILINEAR;
            aniso = 1;
        }

        Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(tfo);
        Ogre::MaterialManager::getSingleton().setDefaultAnisotropy(aniso);
        mDetailsPanel->setParamValue(9, newVal);
    }
    else if (arg.key == OIS::KC_R)   // cycle polygon rendering mode
    {
        Ogre::String newVal;
        Ogre::PolygonMode pm;

        switch (mCamera->getPolygonMode())
        {
        case Ogre::PM_SOLID:
            newVal = "Wireframe";
            pm = Ogre::PM_WIREFRAME;
            break;
        case Ogre::PM_WIREFRAME:
            newVal = "Points";
            pm = Ogre::PM_POINTS;
            break;
        default:
            newVal = "Solid";
            pm = Ogre::PM_SOLID;
        }

        mCamera->setPolygonMode(pm);
        mDetailsPanel->setParamValue(10, newVal);
    }
    else if(arg.key == OIS::KC_F5)   // refresh all textures
    {
        Ogre::TextureManager::getSingleton().reloadAll();
    }
    else if (arg.key == OIS::KC_SYSRQ)   // take a screenshot
    {
        mWindow->writeContentsToTimestampedFile("screenshot", ".jpg");
    }
    else if (arg.key == OIS::KC_ESCAPE)
    {
        mShutDown = true;
    }
	else if(arg.key == OIS::KC_M)
	{
		if(m_cameraControl)
		{
			m_cameraControl = false;
			m_guiRoot->setVisible(true);
			mTrayMgr->showTrays();
		}
		else
		{
			m_cameraControl = true;
			m_guiRoot->setVisible(false);
			mTrayMgr->hideTrays();
		}
	}

	if(m_cameraControl)
	{
		mCameraMan->injectKeyDown(arg);
	}
	else
	{
		CEGUI::System &sys = CEGUI::System::getSingleton();
		sys.injectKeyDown(arg.key);
		sys.injectChar(arg.text);
	}

    return true;
}

bool CartoonHairSimulation::keyReleased( const OIS::KeyEvent &arg )
{
	if(m_cameraControl)
	{
		mCameraMan->injectKeyUp(arg);
	}
	else
	{
		CEGUI::System::getSingleton().injectKeyUp(arg.key);
	}

    return true;
}

bool CartoonHairSimulation::mouseMoved( const OIS::MouseEvent &arg )
{
    //mTrayMgr->injectMouseMove(arg);
	if(m_cameraControl)
	{
		mCameraMan->injectMouseMove(arg);
	}
	else
	{
		CEGUI::System &sys = CEGUI::System::getSingleton();
		sys.injectMouseMove(arg.state.X.rel,arg.state.Y.rel);
		if(arg.state.Z.rel)
		{
			sys.injectMouseWheelChange(arg.state.Z.rel / 120.0f);
		}
	}
    return true;
}

bool CartoonHairSimulation::mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
    //mTrayMgr->injectMouseDown(arg, id);

	if(m_cameraControl)
	{
		mCameraMan->injectMouseDown(arg, id);
	}
	else
	{
		CEGUI::System::getSingleton().injectMouseButtonDown(convertButton(id));
	}

    return true;
}

bool CartoonHairSimulation::mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
    //mTrayMgr->injectMouseUp(arg, id);

	if(m_cameraControl)
	{
		mCameraMan->injectMouseUp(arg, id);
	}
	else
	{
		CEGUI::System::getSingleton().injectMouseButtonUp(convertButton(id));
	}

    return true;
}

//Adjust mouse clipping area
void CartoonHairSimulation::windowResized(Ogre::RenderWindow* rw)
{
    unsigned int width, height, depth;
    int left, top;
    rw->getMetrics(width, height, depth, left, top);

    const OIS::MouseState &ms = mMouse->getMouseState();
    ms.width = width;
    ms.height = height;
}

//Unattach OIS before window shutdown (very important under Linux)
void CartoonHairSimulation::windowClosed(Ogre::RenderWindow* rw)
{
    //Only close for window that created OIS (the main window in these demos)
    if( rw == mWindow )
    {
        if( mInputManager )
        {
            mInputManager->destroyInputObject( mMouse );
            mInputManager->destroyInputObject( mKeyboard );

            OIS::InputManager::destroyInputSystem(mInputManager);
            mInputManager = 0;
        }
    }
}


#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
    INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else
    int main(int argc, char *argv[])
#endif
    {
        // Create application object
        CartoonHairSimulation app;

        try {
            app.go();
        } catch( Ogre::Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
            MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
            std::cerr << "An exception has occured: " <<
                e.getFullDescription().c_str() << std::endl;
#endif
        }

        return 0;
    }

#ifdef __cplusplus
}
#endif
