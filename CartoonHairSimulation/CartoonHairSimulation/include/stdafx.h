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

#include <OISEvents.h>
#include <OISInputManager.h>
#include <OISKeyboard.h>
#include <OISMouse.h>

#include <SdkTrays.h>
#include <SdkCameraMan.h>

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision\CollisionShapes\btShapeHull.h>
#include <BulletCollision\CollisionDispatch\btGhostObject.h>

#include <BulletSoftBody\btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody\btDefaultSoftBodySolver.h>
#include <BulletSoftBody\btSoftBodyHelpers.h>
#include <BulletSoftBody\btSoftBodyRigidBodyCollisionConfiguration.h>

#include <vector>

//https://github.com/atduskgreg/pcl-marching-squares-example/blob/master/marching_cubes.cpp

//#include <pcl/point_types.h>
//#include <pcl/surface/marching_cubes_rbf.h>

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

// TODO: reference additional headers your program requires here