#include "stdafx.h"

#define SHIFT_X 0.2f
#define TOLERANCE 0.000001
#define TEMP_STICTION_THRESHOLD 0.1f
#define TEMP_STICTION_REST_LENGTH 0.08f
#define TEMP_STICTION_K 0.1f

#define ID_INCREMENT 0.05f

#define NUM_HAIR_SAMPLES 5

//http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Collision_Filtering
#define BODY_GROUP 0x1
#define HAIR_GROUP 0x2
#define GHOST_GROUP 0x4
#define SEGMENT_GROUP 0x8

struct HairParameters
{
	std::string directory;
	std::string animation;
	Ogre::SceneManager *sceneMgr;
	btSoftRigidDynamicsWorld *world;
	btSoftBody::Material *edgeMaterial;
	btSoftBody::Material *bendingMaterial;
	btSoftBody::Material *torsionMaterial;
	btSoftBody::Material *stictionMaterial;
	Ogre::Camera *camera;
	Ogre::RenderWindow *window;
	float a;
	float b;
	float c;
	int maxStictionConnections;
	float stictionThreshold;
	float stictionRestLength;
	float stictionK;
	Ogre::Vector3 initialPosition;
	Ogre::Quaternion initialOrientation;
};

struct HairSegment
{
	btSoftBody *strand;
	btGhostObject *ghostObject;
	int node0Index;
	int node1Index;
	std::vector<btSoftBody*> stictionSprings;
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
	//std::vector<std::pair<int,int>> linkedEdges;
};

//http://www.fannieliu.com/hairsim/hairsim.html
class HairModel
{
public:
	HairModel(HairParameters &param);
	~HairModel();
	Ogre::ManualObject* getHairManualObject();
	Ogre::ManualObject* getNormalsManualObject();
	Ogre::ManualObject* getEdgeManualObject();
	Ogre::RenderTexture* getIdBufferTexture();
	//Ogre::BillboardSet* getEdgeBillboardSet();
	void applyHeadTransform(Ogre::Quaternion rotation, Ogre::Vector3 translation);
	void updateManualObject();
	void updateStictionSegments();
	void updateAnchors(float timestep);
	float getSimulationScale();
	void setCurveValues(float a, float b, float c);
private:
	//methods
	Ogre::ColourValue generateUniqueColour();
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
	void generateIdColours();
	void generateVertices(bool update, int section);
	void generateNormals(bool update, int section);
	void addToTempEdgeMap(std::pair<int,int> key, int index1, int index2, int index3);

	void generateEdges(bool update);

	void generateAnchorBody(btSoftRigidDynamicsWorld *world, btSoftBodyWorldInfo &worldInfo, btAlignedObjectArray<btVector3> &points);
	std::string loadAnchorPoints(std::string directory, std::string filename);
	btAlignedObjectArray<btVector3> loadAnchorPositions(std::string filename);
	
	Ogre::Vector3 calculateNormal(Ogre::Vector3 v1, Ogre::Vector3 v2, Ogre::Vector3 v3);
	bool isSilhouette(Edge *edge,int section, Ogre::Vector3 eyeVector);
	void insertSilhouette(std::pair<int,int> element, std::vector<std::pair<int,int>> &temp, std::deque<std::pair<int,int>> &silhouette);
	bool attemptInsert(std::pair<int,int> element, std::deque<std::pair<int,int>> &silhouette);

	//http://stackoverflow.com/questions/13682074/get-2d-screen-point-from-3d-point
	bool toDeviceCoordinates(Ogre::Vector3 &result, Ogre::Vector3 &point,Ogre::Camera *camera);

	//variables

	//physics variables
	float m_simulationScale;
	std::vector<btSoftBody*> m_strandSoftBodies;
	std::vector<btSoftBody*> m_ghostStrandSoftBodies;
	std::vector<Ogre::Vector3> m_hairShape;
	btSoftRigidDynamicsWorld *m_world;

	//stiction springs varialbes
	btSoftBody::Material *m_stictionMaterial;
	int m_maxStictionConnections;
	float m_stictionThreshold;
	float m_stictionRestLength;
	float m_stictionK;
	btAlignedObjectArray<HairSegment*> m_hairSegments;

	//animation variables
	btAlignedObjectArray<btAlignedObjectArray<btVector3>> m_anchorPoints;
	btSoftBody *m_anchors;
	float m_animationTime;
	float m_animationSpeed;
	int m_currentFrame;

	//silhouette variables
	bool m_depthCueCalculated;
	float m_di,m_dc,m_fd;
	Ogre::RenderTexture *m_idBuffer;
	std::vector<Ogre::ColourValue> m_idColours;
	Ogre::ColourValue m_currentId;

	//binding to character variables
	btVector3 m_initialPosition;
	btQuaternion m_initialOrientation;
	
	//rendering variables
	std::vector<Ogre::SimpleSpline> m_hairSplines;
	Ogre::ManualObject *m_hairMesh,*m_normalMesh,*m_edgeMesh;
	//Ogre::BillboardSet *m_edgeSet;
	std::vector<std::vector<Ogre::Vector3>> m_strandVertices;
	std::vector<std::vector<Ogre::Vector3>> m_strandNormals;
	std::vector<int> m_strandIndices;
	float m_a,m_b,m_c;

	Ogre::Camera *m_camera;

	//edge rendering variables - npar2000_lake_et_al.pdf
	std::unordered_map<std::pair<int,int>,Edge,HashFunction,EqualFunction> m_tempEdgeMap;
	std::unordered_map<std::pair<int,int>,Edge,HashFunction,EqualFunction> m_edgeMap;

};