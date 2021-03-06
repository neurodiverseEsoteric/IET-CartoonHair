/*
-----------------------------------------------------------------------------
Filename:    TAMGenerator.cpp
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
#include "TAMGenerator.h"

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#include "../res/resource.h"
#endif


//-------------------------------------------------------------------------------------
TAMGenerator::TAMGenerator(void)
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
    mKeyboard(0)
{
}

//-------------------------------------------------------------------------------------
TAMGenerator::~TAMGenerator(void)
{
    if (mTrayMgr) delete mTrayMgr;
    if (mCameraMan) delete mCameraMan;

    //Remove ourself as a Window listener
    Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
    windowClosed(mWindow);
    delete mRoot;
}

//-------------------------------------------------------------------------------------
bool TAMGenerator::configure(void)
{
    // Show the configuration dialog and initialise the system
    // You can skip this and use root.restoreConfig() to load configuration
    // settings if you were sure there are valid ones saved in ogre.cfg
    if(mRoot->showConfigDialog())
    {
        // If returned true, user clicked OK so initialise
        // Here we choose to let the system create a default rendering window by passing 'true'
        mWindow = mRoot->initialise(true, "TAMGenerator Render Window");
		mWindow->resize(TEX_SIDE,TEX_SIDE);

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
void TAMGenerator::chooseSceneManager(void)
{
    // Get the SceneManager, in this case a generic one
    mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);
}
//-------------------------------------------------------------------------------------
void TAMGenerator::createCamera(void)
{
    // Create the camera
    mCamera = mSceneMgr->createCamera("PlayerCam");

	mCamera->setProjectionType(Ogre::ProjectionType::PT_ORTHOGRAPHIC);
    mCamera->setPosition(Ogre::Vector3(0,0,0.1));
    // Look back along -Z
    mCamera->lookAt(Ogre::Vector3(0,0,-1));
	mCamera->setOrthoWindow(TEX_SIDE+MARGIN,TEX_SIDE+MARGIN);

    mCamera->setNearClipDistance(0.1);
	mCamera->setFarClipDistance(1000);

    mCameraMan = new OgreBites::SdkCameraMan(mCamera);   // create a default camera controller
}
//-------------------------------------------------------------------------------------
void TAMGenerator::createFrameListener(void)
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
    //mTrayMgr->showFrameStats(OgreBites::TL_BOTTOMLEFT);
    //mTrayMgr->showLogo(OgreBites::TL_BOTTOMRIGHT);
    mTrayMgr->hideCursor();

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
void TAMGenerator::destroyScene(void)
{
}
//-------------------------------------------------------------------------------------
void TAMGenerator::createViewports(void)
{
    // Create one viewport, entire window
    Ogre::Viewport* vp = mWindow->addViewport(mCamera);
    vp->setBackgroundColour(Ogre::ColourValue(1,1,1));

    // Alter the camera aspect ratio to match the viewport
    mCamera->setAspectRatio(
        Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
}
//-------------------------------------------------------------------------------------
void TAMGenerator::setupResources(void)
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
void TAMGenerator::createResourceListener(void)
{

}
//-------------------------------------------------------------------------------------
void TAMGenerator::loadResources(void)
{
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}
//-------------------------------------------------------------------------------------
void TAMGenerator::go(void)
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
bool TAMGenerator::setup(void)
{
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

	//createFrameListener();

    return false;
};

float TAMGenerator::getAverageTone(Ogre::PixelBox &pixels)
{
	float sum = 0;
	//get average tone - we will just look at one colour channel (green) as the strokes are b/w
	for(int y = 0 ; y < pixels.getHeight() ; y++)
	{
		for(int x = 0 ; x < pixels.getWidth() ; x++)
		{
			sum += pixels.getColourAt(x,y,0).g;
		}
	}
	return sum/(pixels.getWidth()*pixels.getHeight());
}

Ogre::ManualObject* TAMGenerator::createStroke()
{
	Ogre::ManualObject *strokeQuad = mSceneMgr->createManualObject();
	strokeQuad->setDynamic(true);
	strokeQuad->begin("IETCartoonHair/TAMMaterial",Ogre::RenderOperation::OT_TRIANGLE_LIST);
		
	Ogre::Vector3 topLeft(-STROKE_WIDTH/2.0f,STROKE_HEIGHT/2.0f,0);
	Ogre::Vector3 topRight(STROKE_WIDTH/2.0f,STROKE_HEIGHT/2.0f,0);
	Ogre::Vector3 bottomLeft(-STROKE_WIDTH/2.0f,-STROKE_HEIGHT/2.0f,0);
	Ogre::Vector3 bottomRight(STROKE_WIDTH/2.0f,-STROKE_HEIGHT/2.0f,0);

	strokeQuad->position(topLeft);
	strokeQuad->textureCoord(0,1);

	strokeQuad->position(bottomLeft);
	strokeQuad->textureCoord(0,0);

	strokeQuad->position(bottomRight);
	strokeQuad->textureCoord(1,0);

	strokeQuad->position(topRight);
	strokeQuad->textureCoord(1,1);

	strokeQuad->quad(0,1,2,3);
	strokeQuad->end();

	return strokeQuad;
}

//-------------------------------------------------------------------------------------
void TAMGenerator::createScene(void)
{
	//seed the random number generator
	srand(time(NULL));

	for(int stroke = 0 ; stroke < NUM_STROKES ; stroke++)
	{
		Ogre::ManualObject *strokeQuad = createStroke();
		Ogre::ManualObject *overlapStrokeQuad = createStroke();
		m_strokes.push_back(strokeQuad);
		m_overlapStrokes.push_back(overlapStrokeQuad);
		m_overlapVisible.push_back(false);

		Ogre::SceneNode *node = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		node->attachObject(strokeQuad);
		Ogre::SceneNode *overlapNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		overlapNode->attachObject(overlapStrokeQuad);

		if(stroke>NUM_STROKES/2)
		{
			node->roll(Ogre::Radian(Ogre::Degree(90)));
			overlapNode->roll(Ogre::Radian(Ogre::Degree(90)));
		}

		node->setVisible(false);
		overlapNode->setVisible(false);
		
		m_strokeNodes.push_back(node);
		m_overlapStrokeNodes.push_back(overlapNode);
	}

	//http://www.ogre3d.org/tikiwiki/Intermediate+Tutorial+7
	Ogre::TexturePtr rtt = Ogre::TextureManager::getSingleton().createManual("rtt",Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D,TEX_SIDE,TEX_SIDE,0,Ogre::PF_R8G8B8,Ogre::TU_RENDERTARGET);
	m_renderTex = rtt->getBuffer()->getRenderTarget();
	m_renderTex->addViewport(mCamera);
	m_renderTex->getViewport(0)->setClearEveryFrame(true);
	m_renderTex->getViewport(0)->setBackgroundColour(Ogre::ColourValue::White);

	int pBoxWidth = m_renderTex->getWidth();
	int pBoxHeight = m_renderTex->getHeight();
	Ogre::PixelFormat pBoxPixelFormat = m_renderTex->suggestPixelFormat();
	unsigned char* pBoxPixelData = new unsigned char[pBoxWidth*pBoxHeight*Ogre::PixelUtil::getNumElemBytes(pBoxPixelFormat)];
	
	Ogre::PixelBox pixels(pBoxWidth,pBoxHeight,1,pBoxPixelFormat,pBoxPixelData);

	m_renderTex->update(false);
	int strokeIncrement = NUM_STROKES/NUM_TEXTURES;
	for(int tex = 0 ; tex < NUM_TEXTURES ; tex++)
	{
		for(int stroke = tex*strokeIncrement ; stroke < (tex*strokeIncrement)+strokeIncrement ; stroke++)
		{
			//based on http://www.ogre3d.org/tikiwiki/tiki-index.php?page=Render+Target+to+QImage+%5BQT%5D
			float bestAvgToneDifference = 0;
			Ogre::Vector2 bestPosition(0,0);
			bool overlapEnabled = false;
			Ogre::Vector2 overlapPosition(0,0);

			//get the last average tone
			m_renderTex->copyContentsToMemory(pixels,Ogre::RenderTarget::FB_AUTO);
			float lastTone = getAverageTone(pixels);

			//find the best candidate
			m_strokeNodes[stroke]->setVisible(true);
			std::vector<Ogre::Vector2> overlapStrokePositions;
			std::vector<bool> overlapStrokeEnabled;

			for(int candidate = 0 ; candidate < STROKE_CANDIDATES ; candidate++)
			{
				//generate new position
				Ogre::Real randX = (stroke>NUM_STROKES/2)?
					Ogre::Math::RangeRandom(-(EDGE)+LIMIT_OFFSET+(STROKE_HEIGHT/2.0f),(EDGE)-LIMIT_OFFSET-(STROKE_HEIGHT/2.0f)):
					Ogre::Math::RangeRandom(-(EDGE)+LIMIT_OFFSET,(EDGE)-LIMIT_OFFSET);
				Ogre::Real randY = (stroke>NUM_STROKES/2)?
					Ogre::Math::RangeRandom(-(EDGE)+LIMIT_OFFSET,(EDGE)-LIMIT_OFFSET):
					Ogre::Math::RangeRandom(-(EDGE)+LIMIT_OFFSET+(STROKE_HEIGHT/2.0f),(EDGE)-LIMIT_OFFSET-(STROKE_HEIGHT/2.0f));
				m_strokeNodes[stroke]->setPosition(randX,randY,0);

				//create overlap strokes if necessary
				overlapStrokePositions.push_back(Ogre::Vector2(randX,randY));
				overlapStrokeEnabled.push_back(false);

				//if vertical strokes - we are looking at the y axis
				if(stroke>NUM_STROKES/2)
				{
					//create new stroke object on the bottom
					if(randY+(STROKE_WIDTH/2.0f) > EDGE)
					{
						overlapStrokeEnabled[candidate] = true;
						float offset = Ogre::Math::Abs((EDGE)-randY);
						overlapStrokePositions[candidate].y = -(EDGE)+offset;
					}
					//create new stroke object on the top
					else if(randY-(STROKE_WIDTH/2.0f) < -(EDGE))
					{
						overlapStrokeEnabled[candidate] = true;
						float offset = Ogre::Math::Abs(-(EDGE)-randY);
						overlapStrokePositions[candidate].y = (EDGE)-offset;
					}
				}
				//if horizontal strokes - we are looking at the x axis
				else
				{
					//create new stroke object on the left
					if(randX+(STROKE_WIDTH/2.0f) > (EDGE))
					{
						overlapStrokeEnabled[candidate] = true;
						float offset = Ogre::Math::Abs((EDGE)-randX);
						overlapStrokePositions[candidate].x = -(EDGE)+offset;
					}
					//create new stroke object on the right
					else if(randX-(STROKE_WIDTH/2.0f) < -(EDGE))
					{
						overlapStrokeEnabled[candidate] = true;
						float offset = Ogre::Math::Abs(-(EDGE)-randX);
						overlapStrokePositions[candidate].x = (EDGE)-offset;
					}
				}

				m_overlapStrokeNodes[stroke]->setVisible(overlapStrokeEnabled[candidate]);
				m_overlapStrokeNodes[stroke]->setPosition(
					overlapStrokePositions[candidate].x,
					overlapStrokePositions[candidate].y,
					0);

				//determine contribution
				m_renderTex->update(false);
				m_renderTex->copyContentsToMemory(pixels,Ogre::RenderTarget::FB_AUTO);
				float newTone = getAverageTone(pixels);

				float difference = Ogre::Math::Abs(newTone-lastTone); //if i was using different levels this would be sum of all levels(newTone-lastTone)
				if(difference>bestAvgToneDifference)
				{
					bestAvgToneDifference = difference;
					bestPosition.x = randX;
					bestPosition.y = randY;
					overlapEnabled = overlapStrokeEnabled[candidate];
					overlapPosition = overlapStrokePositions[candidate];
				}
			}

			//candidate found
			m_strokeNodes[stroke]->setPosition(bestPosition.x,bestPosition.y,0);
			m_overlapStrokeNodes[stroke]->setPosition(overlapPosition.x,overlapPosition.y,0);
			m_overlapStrokeNodes[stroke]->setVisible(overlapEnabled);
			m_overlapVisible[stroke] = overlapEnabled;
		}

		m_renderTex->update(false);
		//calculate name
		std::stringstream name;
		name << "textures\\full\\tamtex" << tex << ".png";
		m_renderTex->writeContentsToFile(name.str());
	}

	//now to render the mip maps
	int side = TEX_SIDE;
	int visibleIncrement = 2;
	for(int level = 0 ; level < MIPMAP_LEVELS ; level++)
	{
		if(level>0)
		{
			visibleIncrement *= 2;
		}
		side = side/2;
		//change render target size

		rtt = Ogre::TextureManager::getSingleton().createManual("rtt",Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D,side,side,0,Ogre::PF_R8G8B8,Ogre::TU_RENDERTARGET);
		m_renderTex = rtt->getBuffer()->getRenderTarget();
		m_renderTex->addViewport(mCamera);
		m_renderTex->getViewport(0)->setClearEveryFrame(true);
		m_renderTex->getViewport(0)->setBackgroundColour(Ogre::ColourValue::White);

		//go through all nodes and increase their scale to keep the pixel sizes the same
		for(int stroke = 0 ; stroke < m_strokeNodes.size() ; stroke++)
		{
			m_strokeNodes[stroke]->setVisible(false);
			m_overlapStrokeNodes[stroke]->setVisible(false);
			//update stroke
			Ogre::ManualObject *object = m_strokes[stroke];
			object->beginUpdate(0);
			float height = STROKE_HEIGHT*visibleIncrement;
			Ogre::Vector3 topLeft(-STROKE_WIDTH/2.0f,height/2.0f,0);
			Ogre::Vector3 topRight(STROKE_WIDTH/2.0f,height/2.0f,0);
			Ogre::Vector3 bottomLeft(-STROKE_WIDTH/2.0f,-height/2.0f,0);
			Ogre::Vector3 bottomRight(STROKE_WIDTH/2.0f,-height/2.0f,0);

			object->position(topLeft);
			object->textureCoord(0,1);

			object->position(bottomLeft);
			object->textureCoord(0,0);

			object->position(bottomRight);
			object->textureCoord(1,0);

			object->position(topRight);
			object->textureCoord(1,1);

			object->quad(0,1,2,3);
			object->end();
		}

		for(int tex = 0 ; tex < NUM_TEXTURES ; tex++)
		{
			int count = 0;
			for(int stroke = tex*strokeIncrement ; stroke < (tex*strokeIncrement)+strokeIncrement ; stroke++)
			{
				count++;
				if(count==visibleIncrement)
				{
					count = 0;
					m_strokeNodes[stroke]->setVisible(true);
					m_overlapStrokeNodes[stroke]->setVisible(m_overlapVisible[stroke]);
				}
			}

			m_renderTex->update(false);
			//calculate name
			std::stringstream name;
			name << "textures\\mip\\tamtex" << tex << "mip" << level << ".png";
			m_renderTex->writeContentsToFile(name.str());
		}
	}
}
//-------------------------------------------------------------------------------------
bool TAMGenerator::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    if(mWindow->isClosed())
        return false;

    if(mShutDown)
        return false;

    //Need to capture/update each device
    mKeyboard->capture();
    mMouse->capture();

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

    return true;
}
//-------------------------------------------------------------------------------------
bool TAMGenerator::keyPressed( const OIS::KeyEvent &arg )
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
            mTrayMgr->moveWidgetToTray(mDetailsPanel, OgreBites::TL_TOPRIGHT, 0);
            mDetailsPanel->show();
        }
        else
        {
            mTrayMgr->removeWidgetFromTray(mDetailsPanel);
            mDetailsPanel->hide();
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

    //mCameraMan->injectKeyDown(arg);
    return true;
}

bool TAMGenerator::keyReleased( const OIS::KeyEvent &arg )
{
    //mCameraMan->injectKeyUp(arg);
    return true;
}

bool TAMGenerator::mouseMoved( const OIS::MouseEvent &arg )
{
    if (mTrayMgr->injectMouseMove(arg)) return true;
    //mCameraMan->injectMouseMove(arg);
    return true;
}

bool TAMGenerator::mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
    if (mTrayMgr->injectMouseDown(arg, id)) return true;
    //mCameraMan->injectMouseDown(arg, id);
    return true;
}

bool TAMGenerator::mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
    if (mTrayMgr->injectMouseUp(arg, id)) return true;
    //mCameraMan->injectMouseUp(arg, id);
    return true;
}

//Adjust mouse clipping area
void TAMGenerator::windowResized(Ogre::RenderWindow* rw)
{
    unsigned int width, height, depth;
    int left, top;
    rw->getMetrics(width, height, depth, left, top);

    const OIS::MouseState &ms = mMouse->getMouseState();
    ms.width = width;
    ms.height = height;
}

//Unattach OIS before window shutdown (very important under Linux)
void TAMGenerator::windowClosed(Ogre::RenderWindow* rw)
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
        TAMGenerator app;

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
