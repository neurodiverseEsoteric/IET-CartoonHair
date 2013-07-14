#include "stdafx.h"

//#define IMAGE_SPACE_HATCHING

#define EDGE_SPRINGS
#define BENDING_SPRINGS
#define TORSION_SPRINGS
#define ANCHOR_SPRINGS
#define GHOST_STRAND
//#define ANGLE_SCALING
//#define DEPTH_SCALING
//#define CONSTANT_BLENDING_SPRINGS
#define BLENDING_QUADRATIC_A 1.0
#define BLENDING_QUADRATIC_B 0
#define BLENDING_QUADRATIC_C 0

#define SHIFT_X 0.2f
#define TOLERANCE 0.000001

#define ID_INCREMENT 0.05f
#define ANCHOR_FRAME_INCRMENT 0.01f

#define NUM_HAIR_SAMPLES 5
#define NUM_HAIR_SHAPE_SAMPLES 6

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
	btVector3 initialPosition;
	btQuaternion initialOrientation;
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
	HairModel(std::string directory, std::string animation, Ogre::Camera *camera, Ogre::RenderWindow *window, Ogre::SceneManager *sceneMgr,
		btSoftBody::Material *edgeMaterial, btSoftBody::Material *torsionMaterial, btSoftBody::Material *bendingMaterial,btSoftBody::Material *anchorMaterial,
		btSoftRigidDynamicsWorld *world, float a,float b, float c);//HairParameters &param);
	~HairModel();
	Ogre::ManualObject* getHairManualObject();
	Ogre::ManualObject* getNormalsManualObject();
	Ogre::ManualObject* getEdgeManualObject();
	Ogre::ManualObject* getDebugEdgesManualObject();

	Ogre::RenderTexture* getIdBufferTexture();

	void applyHeadTransform(bool first, Ogre::Vector3 translation, Ogre::Quaternion rotation);
	void updateManualObject();
	void updateAnchors(float timestep);
	float getSimulationScale();
	void setCurveValues(float a, float b, float c);
private:
	//methods
	Ogre::ColourValue generateUniqueColour();

	void generateHairStrands(std::string filename,btSoftRigidDynamicsWorld *world,
		btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial,btSoftBody::Material *anchorMaterial);
	void generateHairMesh(Ogre::SceneManager *sceneMgr);
	float determineScale(float x);
	Ogre::Quaternion determineRotation(Ogre::Vector3 up, Ogre::Vector3 node0, Ogre::Vector3 node1);
	void createOrUpdateManualObject(bool update);
	btSoftBody *createHairStrand(int strandIndex, btSoftRigidDynamicsWorld *world, btAlignedObjectArray<btVector3> &particles, std::vector<float> &masses, btSoftBodyWorldInfo &worldInfo,
		btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial,btSoftBody::Material *anchorMaterial);
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
	Ogre::Vector3 toWorldCoordinates(Ogre::Vector3 &point, Ogre::Camera *camera);

	//variables

	//physics variables
	float m_simulationScale;
	std::vector<btSoftBody*> m_strandSoftBodies;
	std::vector<btSoftBody*> m_ghostStrandSoftBodies;
	std::vector<Ogre::Vector3> m_hairShape;
	btSoftRigidDynamicsWorld *m_world;

	//animation variables
	btAlignedObjectArray<btVector3> m_rootPoints;
	btAlignedObjectArray<btVector3> m_anchorPoints;
	btAlignedObjectArray<btSoftBody::Material *> m_blendingSpringMaterials;
	std::vector<Ogre::SimpleSpline> m_anchorSplines;
	btSoftBody *m_anchors;
	float m_animationTime;
	float m_animationSpeed;
	bool m_positiveInterpolation;

	//temporally coherent hatching variables

	//silhouette variables
	Ogre::RenderTexture *m_idBuffer;
	std::vector<Ogre::ColourValue> m_idColours;
	Ogre::ColourValue m_currentId;

	//binding to character variables
	Ogre::Vector3 m_translationOffset;
	Ogre::Quaternion m_orientationOffset;
	
	//rendering variables
	std::vector<Ogre::SimpleSpline> m_hairSplines;
	Ogre::ManualObject *m_hairMesh,*m_normalMesh,*m_edgeMesh,*m_debugEdges;
	//Ogre::BillboardSet *m_edgeSet;
	std::vector<std::vector<Ogre::Vector3>> m_strandVertices;
	std::vector<std::vector<Ogre::Vector3>> m_strandNormals;
	std::vector<Ogre::Vector2> m_strandTextureCoordinates;
	std::vector<int> m_strandIndices;
	float m_a,m_b,m_c;

	Ogre::Camera *m_camera;

	//edge rendering variables - npar2000_lake_et_al.pdf
	std::unordered_map<std::pair<int,int>,Edge,HashFunction,EqualFunction> m_tempEdgeMap;
	std::unordered_map<std::pair<int,int>,Edge,HashFunction,EqualFunction> m_edgeMap;

};