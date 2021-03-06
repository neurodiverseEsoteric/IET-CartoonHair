// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreLogManager.h>
#include <OgreRoot.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>
#include <OgreException.h>
#include <OgreWindowEventUtilities.h>
#include <OgreSimpleSpline.h>
#include <OgreTagPoint.h>
#include <OgreFont.h>
#include <OgreFontManager.h>

#include <OISEvents.h>
#include <OISInputManager.h>
#include <OISKeyboard.h>
#include <OISMouse.h>

#include <SdkTrays.h>
#include <SdkCameraMan.h>

#include <CEGUI.h>
#include <RendererModules\Ogre\CEGUIOgreRenderer.h>

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision\CollisionShapes\btShapeHull.h>
#include <BulletCollision\CollisionDispatch\btGhostObject.h>
#include <BulletCollision\CollisionShapes\btConvexPointCloudShape.h>

#include <BulletSoftBody\btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody\btDefaultSoftBodySolver.h>
#include <BulletSoftBody\btSoftBodyHelpers.h>
#include <BulletSoftBody\btSoftBodyRigidBodyCollisionConfiguration.h>

#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtx\rotate_vector.hpp>
#include <glm\gtc\type_ptr.hpp>

#include <vector>
#include <unordered_map>
#include <string>
#include <limits>
#include <algorithm>

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

// TODO: reference additional headers your program requires here