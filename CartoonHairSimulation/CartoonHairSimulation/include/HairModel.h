#include "stdafx.h"

#define SHIFT_X 0.2f
#define TOLERANCE 0.000001
#define TEMP_STICTION_THRESHOLD 0.1f
#define TEMP_STICTION_REST_LENGTH 0.08f
#define TEMP_STICTION_K 0.1f

//http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Collision_Filtering
#define BODY_GROUP 0x1
#define HAIR_GROUP 0x2
#define GHOST_GROUP 0x4
#define SEGMENT_GROUP 0x8

struct HairSegment
{
	btSoftBody *strand;
	btGhostObject *ghostObject;
	int node0Index;
	int node1Index;
};

//http://www.fannieliu.com/hairsim/hairsim.html
class HairModel
{
public:
	HairModel(const char* filename, Ogre::SceneManager *sceneMgr, btSoftRigidDynamicsWorld *world,
		btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial);
	~HairModel();
	Ogre::ManualObject* getHairManualObject();
	Ogre::ManualObject* getNormalsManualObject();
	void updateManualObject();
	void updateStictionSegments();
	float getSimulationScale();
private:
	//methods
	void getClosestPoints(const btVector3 &strand0p0,const btVector3 &strand0p1, const btVector3 &strand1p0, const btVector3 &strand1p1, btVector3 &point0, btVector3 &point1);
	void addStictionSegment(btSoftRigidDynamicsWorld *world, btSoftBody* strand, int nodeIndex0, int nodeIndex1);
	void generateHairStrands(const char* filename,btSoftRigidDynamicsWorld *world,
		btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial);
	void generateHairMesh(Ogre::SceneManager *sceneMgr);
	float determineScale(float x);
	Ogre::Quaternion determineRotation(Ogre::Vector3 up, Ogre::Vector3 node0, Ogre::Vector3 node1);
	void createOrUpdateManualObject(bool update);
	btSoftBody *createHairStrand(btSoftRigidDynamicsWorld *world, btAlignedObjectArray<btVector3> &particles, std::vector<float> &masses, btSoftBodyWorldInfo &worldInfo,
		btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial);
	btSoftBody *createAndLinkGhostStrand(btSoftBody *strand,
		btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial);

	//variables

	//physics variables
	float m_simulationScale;
	std::vector<btSoftBody*> m_strandSoftBodies;
	std::vector<btSoftBody*> m_ghostStrandSoftBodies;
	std::vector<Ogre::Vector3> m_hairShape;
	btAlignedObjectArray<HairSegment*> m_hairSegments;
	
	//rendering variables
	Ogre::ManualObject *m_hairMesh, *m_normalMesh;
	std::vector<std::vector<Ogre::Vector3>> m_strandVertices;
	std::vector<std::vector<Ogre::Vector3>> m_strandNormals;
	std::vector<int> m_strandIndices;
};