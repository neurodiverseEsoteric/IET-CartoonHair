#include "stdafx.h"

#define SHIFT_X 0.2f

//http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Collision_Filtering
#define BODY_GROUP 0x1
#define HAIR_GROUP 0x2
#define GHOST_GROUP 0x4

class HairModel
{
public:
	HairModel(const char* filename, Ogre::SceneManager *sceneMgr, btSoftRigidDynamicsWorld *world);
	~HairModel();
	Ogre::ManualObject* getManualObject();
	void updateManualObject();
	float getSimulationScale();
private:
	float determineScale(float x);
	void createOrUpdateManualObject(bool update);
	std::vector<btSoftBody*> m_strandSoftBodies;
	std::vector<btSoftBody*> m_ghostStrandSoftBodies;
	std::vector<Ogre::Vector3> m_hairShape;
	float m_simulationScale;
	Ogre::ManualObject *m_hairMesh;
	btSoftBody *createHairStrand(btAlignedObjectArray<btVector3> &particles, std::vector<float> &masses, btSoftBodyWorldInfo &worldInfo);
	btSoftBody *createAndLinkGhostStrand(btSoftBody *strand);
};