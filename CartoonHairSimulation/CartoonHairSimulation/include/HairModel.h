#include "stdafx.h"

class HairModel
{
public:
	HairModel(const char* filename, Ogre::SceneManager *sceneMgr, btSoftBodyWorldInfo &worldInfo, btSoftRigidDynamicsWorld *world);
	~HairModel();
private:
	std::vector<btSoftBody*> m_strandSoftBodies; 
	Ogre::ManualObject *m_hairMesh;

	btSoftBody *createHairStrand(std::vector<btVector3> &particles, std::vector<float> &masses, btSoftBodyWorldInfo *worldInfo);
};