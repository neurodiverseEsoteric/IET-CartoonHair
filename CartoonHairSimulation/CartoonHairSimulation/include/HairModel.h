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

enum EdgeType
{
	NOTHING, BORDER, CREASE, SILHOUETTE
};

//http://stackoverflow.com/questions/2099540/defining-custom-hash-function-and-equality-function-for-unordered-map
struct HashFunction
{
	std::size_t operator() (const std::pair<int,int>& key) const
	{
		return key.first + key.second;
	}
};

struct EqualFunction
{
	bool operator() (const std::pair<int,int>& lhs,const std::pair<int,int>& rhs) const
	{
		if((lhs.first == rhs.first && lhs.second == rhs.second)||
			(lhs.first == rhs.second && lhs.second == rhs.first))
		{
			return true;
		}
		else return false;
	}
};

struct Edge
{
	Edge()
	{
		flag = EdgeType::NOTHING;
		edgeCount = 0;
	}
	EdgeType flag;
	int edgeCount;
	int triangle1Indices[3];
	int triangle2Indices[3];
};

//http://www.fannieliu.com/hairsim/hairsim.html
class HairModel
{
public:
	HairModel(std::string directory, std::string animation, Ogre::SceneManager *sceneMgr, btSoftRigidDynamicsWorld *world,
		btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial,
		Ogre::Camera *camera,
		float a, float b, float c);
	~HairModel();
	Ogre::ManualObject* getHairManualObject();
	Ogre::ManualObject* getNormalsManualObject();
	Ogre::ManualObject* getEdgeManualObject();
	//Ogre::BillboardSet* getEdgeBillboardSet();
	void updateManualObject();
	void updateStictionSegments();
	void updateAnchors(float timestep);
	float getSimulationScale();
	void setCurveValues(float a, float b, float c);
private:
	//methods
	void getClosestPoints(const btVector3 &strand0p0,const btVector3 &strand0p1, const btVector3 &strand1p0, const btVector3 &strand1p1, btVector3 &point0, btVector3 &point1);
	void addStictionSegment(btSoftRigidDynamicsWorld *world, btSoftBody* strand, int nodeIndex0, int nodeIndex1);

	void generateHairStrands(std::string filename,btSoftRigidDynamicsWorld *world,
		btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial);
	void generateHairMesh(Ogre::SceneManager *sceneMgr);
	float determineScale(float x);
	Ogre::Quaternion determineRotation(Ogre::Vector3 up, Ogre::Vector3 node0, Ogre::Vector3 node1);
	void createOrUpdateManualObject(bool update);
	btSoftBody *createHairStrand(int strandIndex, btSoftRigidDynamicsWorld *world, btAlignedObjectArray<btVector3> &particles, std::vector<float> &masses, btSoftBodyWorldInfo &worldInfo,
		btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial);
	btSoftBody *createAndLinkGhostStrand(btSoftBody *strand,
		btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial);

	void generateIndices();
	void generateEdgeMap();
	void generateVertices(bool update, int section);
	void generateNormals(bool update, int section);
	void addToEdgeMap(std::pair<int,int> key, int index1, int index2, int index3);

	void generateEdges(bool update);

	void generateAnchorBody(btSoftRigidDynamicsWorld *world, btSoftBodyWorldInfo &worldInfo, btAlignedObjectArray<btVector3> &points);
	std::string loadAnchorPoints(std::string directory, std::string filename);
	btAlignedObjectArray<btVector3> loadAnchorPositions(std::string filename);
	
	Ogre::Vector3 calculateNormal(Ogre::Vector3 v1, Ogre::Vector3 v2, Ogre::Vector3 v3);

	//variables

	//physics variables
	float m_simulationScale;
	std::vector<btSoftBody*> m_strandSoftBodies;
	std::vector<btSoftBody*> m_ghostStrandSoftBodies;
	std::vector<Ogre::Vector3> m_hairShape;
	btAlignedObjectArray<HairSegment*> m_hairSegments;
	btSoftRigidDynamicsWorld *m_world;

	//animation variables
	btAlignedObjectArray<btAlignedObjectArray<btVector3>> m_anchorPoints;
	btSoftBody *m_anchors;
	float m_animationTime;
	float m_animationSpeed;
	int m_currentFrame;
	
	//rendering variables
	Ogre::ManualObject *m_hairMesh,*m_normalMesh,*m_edgeMesh;
	//Ogre::BillboardSet *m_edgeSet;
	std::vector<std::vector<Ogre::Vector3>> m_strandVertices;
	std::vector<std::vector<Ogre::Vector3>> m_strandNormals;
	std::vector<int> m_strandIndices;
	float m_a,m_b,m_c;

	Ogre::Camera *m_camera;

	//edge rendering variables - npar2000_lake_et_al.pdf
	std::unordered_map<std::pair<int,int>,Edge,HashFunction,EqualFunction> m_edgeMap;

};