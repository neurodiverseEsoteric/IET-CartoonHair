#include "stdafx.h"

//edge types used by the silhouette detection
enum EdgeType
{
	NOTHING, BORDER, CREASE, SILHOUETTE
};

//hash function for silhouette edge hash table
struct HashFunction
{
	std::size_t operator() (const std::pair<int,int>& key) const
	{
		return key.first + key.second;
	}
};

//function used by silhouette edge hash table to determine if entries are identical
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

//silhouette edge parameters
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

enum SortType {XSORT,YSORT,ZSORT};
//based on http://www.cplusplus.com/reference/algorithm/sort/
//i have decided to use x to sort as the highlights are typically seen horizontally
struct HighlightSort
{
	SortType sortType;
	bool operator() (Ogre::Vector3 i, Ogre::Vector3 j)
	{
		if(sortType == SortType::XSORT)
		{
			return i.x < j.x;
		}
		else if(sortType == SortType::YSORT)
		{
			return i.y < j.y;
		}
		else
		{
			return i.z < j.z;
		}
	}
};

/*
This class is the core of the project application. It is responsible for generating the hair strands, adding them to the physics simulation
and rendering them.
*/
class HairModel
{
public:
	HairModel(std::string directory, std::string animation, Ogre::Camera *camera, Ogre::Light *light, Ogre::RenderWindow *window, Ogre::SceneManager *sceneMgr,
		btSoftRigidDynamicsWorld *world, float hairA,float hairB, float hairC, float blendingA, float blendingB, float blendingC,
		float edgeStiffness, float bendingStiffness, float torsionStiffness, float anchorStiffness);
	~HairModel();

	Ogre::ManualObject* getHairManualObject();
	Ogre::ManualObject* getNormalsManualObject();
	Ogre::ManualObject* getEdgeManualObject();
	Ogre::ManualObject* getHighlightManualObject();
	Ogre::ManualObject* getDebugEdgesManualObject();
	Ogre::RenderTexture* getIdBufferTexture();

	void applyHeadTransform(bool first, Ogre::Vector3 translation, Ogre::Quaternion rotation);
	void updateManualObject();
	void updateAnchors(float timestep);
	void updateCollisionMargins(float min,float max);
	float getSimulationScale();
	void setCurveValues(float a, float b, float c);

	void enableBlinnSpecular(bool value);
	void enableSpecularTexture(bool value);
	void enableBacklightingTexture(bool value);
	void enableDepthDetailAxis(bool value);
	void enableVariableSilhouetteIntensity(bool value);
	void enableSobel(bool value);
	void enableHatching(bool value);
	void enableSimpleHatching(bool value);
	void enableAngleScaling(bool value);
	void enableDepthScaling(bool value);
	void setSilhouetteStrokeScale(float value);
	void setSilhouetteStrokeLimit(float value);
	void setZMin(float value);
	void setZScale(float value);
	void setBlinnS(float value);
	void setSpecularTextureS(float value);
	void setBacklightingS(float value);
	void setHairColour(Ogre::Vector4 value);
	void setStrokeScale(float value);
	void setEdgeSpringStiffness(float value);
	void setBendingSpringStiffness(float value);
	void setTorsionSpringStiffness(float value);
	void setBlendingSpringStiffness(float value);
	void setBlendingCurve(float a,float b, float c);
	void setSilhouetteCurve(float a, float b, float c);
	void rebuildMesh(int hairResolution, int shapeResolution);
	
private:
	//methods
	Ogre::ColourValue generateUniqueColour();
	void generateHairStrands(std::string filename,btSoftRigidDynamicsWorld *world,
		btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial,float anchorStiffness);
	void generateHairMesh(Ogre::SceneManager *sceneMgr);
	float determineScale(float x);
	Ogre::Quaternion determineRotation(Ogre::Vector3 up, Ogre::Vector3 node0, Ogre::Vector3 node1);
	void createOrUpdateManualObject(bool update);
	btSoftBody *createHairStrand(int strandIndex, btSoftRigidDynamicsWorld *world, btAlignedObjectArray<btVector3> &particles, std::vector<float> &masses, btSoftBodyWorldInfo &worldInfo,
		btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial);
	btSoftBody *createAndLinkGhostStrand(btSoftBody *strand,
		btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial);

	void generateHairShape();
	void updateLinks();
	void generateIndices();
	void generateEdgeMap();
	void generateIdColours();
	void generateVertices(bool update, int section);
	void generateNormals(bool update, int section);
	void generateEdges(bool update);
	void generateSpecularHighlights(bool update);
	void addToTempEdgeMap(std::pair<int,int> key, int index1, int index2, int index3);
	void generateQuadStrip(std::vector<Ogre::Vector3> &screenSpacePoints,std::vector<Ogre::Vector3> &points, float &zMin, float &zMax, float scaleFactor);

	void generateAnchorBody(btSoftRigidDynamicsWorld *world, btSoftBodyWorldInfo &worldInfo, btAlignedObjectArray<btVector3> &points);
	std::string loadAnchorPoints(std::string directory, std::string filename);
	btAlignedObjectArray<btVector3> loadAnchorPositions(std::string filename);
	
	Ogre::Vector3 calculateNormal(Ogre::Vector3 v1, Ogre::Vector3 v2, Ogre::Vector3 v3);
	bool isSilhouette(Edge *edge,int section, Ogre::Vector3 eyeVector);
	void insertSilhouette(std::pair<int,int> element, std::vector<std::pair<int,int>> &temp, std::deque<std::pair<int,int>> &silhouette);
	bool attemptInsert(std::pair<int,int> element, std::deque<std::pair<int,int>> &silhouette);

	//http://stackoverflow.com/questions/13682074/get-2d-screen-point-from-3d-point
	Ogre::Vector3 toDeviceCoordinates(Ogre::Vector3 &point,Ogre::Camera *camera);
	Ogre::Vector3 toWorldCoordinates(Ogre::Vector3 &point, Ogre::Camera *camera);

	//variables
	//physics variables
	float m_simulationScale, m_blendingA, m_blendingB, m_blendingC;
	std::vector<btSoftBody*> m_strandSoftBodies;
	std::vector<btSoftBody*> m_ghostStrandSoftBodies;
	std::vector<Ogre::Vector3> m_hairShape;
	btSoftRigidDynamicsWorld *m_world;
	btSoftBody::Material *m_edgeMaterial, *m_torsionMaterial, *m_bendingMaterial;

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
	Ogre::ManualObject *m_hairMesh,*m_normalMesh,*m_edgeMesh,*m_highlightMesh,*m_debugEdges;
	//Ogre::BillboardSet *m_edgeSet;
	std::vector<std::vector<Ogre::Vector3>> m_strandVertices;
	std::vector<std::vector<Ogre::Vector3>> m_strandNormals;
	std::vector<Ogre::Vector2> m_strandTextureCoordinates;
	std::vector<int> m_strandIndices;
	float m_a,m_b,m_c;

	Ogre::Camera *m_camera;
	Ogre::Light *m_light;

	//edge rendering variables - npar2000_lake_et_al.pdf
	std::unordered_map<std::pair<int,int>,Edge,HashFunction,EqualFunction> m_tempEdgeMap;
	std::unordered_map<std::pair<int,int>,Edge,HashFunction,EqualFunction> m_edgeMap;

	//highlight variables
	HighlightSort m_candidateSort;

	//effects variables
	bool m_blinnSpecularEnabled,m_specularTextureEnabled,m_backlightingTextureEnabled,m_depthDetailAxisEnabled;
	bool m_variableSilhouetteIntensity, m_sobelEnabled, m_hatchingEnabled, m_simpleHatching;
	bool m_angleScalingEnabled, m_depthScalingEnabled;
	float m_zMin,m_zScale, m_blinnS, m_specTexS, m_backlightS, m_strokeScale, m_silhouetteStrokeScale, m_silhouetteStrokeLimit;
	float m_silhouetteA,m_silhouetteB,m_silhouetteC;
	Ogre::Vector4 m_hairColour;
	int m_hairResolution, m_shapeResolution;

};