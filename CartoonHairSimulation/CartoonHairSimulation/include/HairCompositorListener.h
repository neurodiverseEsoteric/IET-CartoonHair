#include "stdafx.h"
//http://www.ogre3d.org/docs/api/html/classOgre_1_1CompositorInstance_1_1Listener.html
class HairCompositorListener : public Ogre::CompositorInstance::Listener
{
public:
	HairCompositorListener(int width, int height, float edgeThreshold, int xDilation, int yDilation)
	{
		m_width = width;
		m_height = height;
		m_edgeThreshold = edgeThreshold;
		m_xDilation = xDilation;
		m_yDilation = yDilation;
	}

	virtual ~HairCompositorListener()
	{

	}

	void setScreenDimensions(int width, int height)
	{
		m_width = width;
		m_height = height;
	}

	void setEdgeThreshold(float value)
	{
		m_edgeThreshold = value;
	}

	void setDilationValues(int x,int y)
	{
		m_xDilation = x;
		m_yDilation = y;
	}

private:
	float m_width,m_height,m_edgeThreshold;
	int m_xDilation, m_yDilation;

	virtual void notifyMaterialSetup(Ogre::uint32 passId, Ogre::MaterialPtr &mat)
	{

	}

	virtual void notifyMaterialRender(Ogre::uint32 passId, Ogre::MaterialPtr &mat)
	{
		if(mat->getName().compare("c0/IETCartoonHair/Sobel") == 0)
		{
			Ogre::GpuProgramParametersSharedPtr param = mat->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
			param->setNamedConstant("width",m_width);
			param->setNamedConstant("height",m_height);
			param->setNamedConstant("edgeThreshold",m_edgeThreshold);
		}
		else if(mat->getName().compare("c1/IETCartoonHair/Dilation") == 0)
		{
			Ogre::GpuProgramParametersSharedPtr param = mat->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
			param->setNamedConstant("width",m_width);
			param->setNamedConstant("height",m_height);
			param->setNamedConstant("dilationX",m_xDilation);
			param->setNamedConstant("dilationY",m_yDilation);
		}
	}

	virtual void notifyResourcesCreated(bool forResizeOnly)
	{

	}
};