#include "stdafx.h"

#define SHIFT_X 0.2f

//http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Collision_Filtering
#define BODY_GROUP 0x1
#define HAIR_GROUP 0x2
#define GHOST_GROUP 0x4

//struct HairSegment
//{
//	btSoftBody::Node* node0;
//	btSoftBody::Node* node1;
//	btDbvtNode* leaf;
//};

struct BVHStrand
{
	btSoftBody* strand;
	btDbvtNode* node;
	btDbvtAabbMm vol;
};


//http://www.fannieliu.com/hairsim/hairsim.html
class HairModel
{
public:
	HairModel(const char* filename, Ogre::SceneManager *sceneMgr, btSoftRigidDynamicsWorld *world,
		btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial);
	~HairModel();
	Ogre::ManualObject* getManualObject();
	void updateManualObject();
	//void updateStictionSegments();
	void updateStrandBVH();
	float getSimulationScale();
private:
	//methods
	//btDbvtAabbMm calculateAABB(HairSegment *segment);
	//void addStictionSegment(btSoftBody* strand, int nodeIndex0, int nodeIndex1);
	void calculateAABB(BVHStrand *strand);
	void generateHairStrands(const char* filename,btSoftRigidDynamicsWorld *world,
		btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial);
	void generateHairMesh(Ogre::SceneManager *sceneMgr);
	float determineScale(float x);
	Ogre::Quaternion determineRotation(Ogre::Vector3 up, Ogre::Vector3 node0, Ogre::Vector3 node1);
	void createOrUpdateManualObject(bool update);
	btSoftBody *createHairStrand(btAlignedObjectArray<btVector3> &particles, std::vector<float> &masses, btSoftBodyWorldInfo &worldInfo,
		btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial);
	btSoftBody *createAndLinkGhostStrand(btSoftBody *strand,
		btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial);

	//variables

	//physics variables
	float m_simulationScale;
	std::vector<btSoftBody*> m_strandSoftBodies;
	std::vector<btSoftBody*> m_ghostStrandSoftBodies;
	std::vector<Ogre::Vector3> m_hairShape;
	//btDbvt* m_segmentBVH;
	//btAlignedObjectArray<HairSegment*> m_hairSegments;
	btDbvt* m_strandBVH;
	btAlignedObjectArray<BVHStrand*> m_BVHStrands;
	
	//rendering variables
	Ogre::ManualObject *m_hairMesh;
	std::vector<std::vector<Ogre::Vector3>> m_strandVertices;
	std::vector<std::vector<Ogre::Vector3>> m_strandNormals;
	std::vector<int> m_strandIndices;
};