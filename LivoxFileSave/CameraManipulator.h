#ifndef _CAMERAMANIPULATOR_H_
#define _CAMERAMANIPULATOR_H_
#include <osgGA/TrackballManipulator>

class TrackballManipulatorEx : public osgGA::TrackballManipulator 
{
public:
	TrackballManipulatorEx(int flags = DEFAULT_SETTINGS);

	bool performMovementLeftMouseButton( const double eventTimeDelta, const double dx, const double dy );		
	bool performMovementRightMouseButton( const double eventTimeDelta, const double dx, const double dy );
	void setMinimumDistance( const double& minimumDistance, bool relativeToModelSize = false );
	void setNode( osg::Node* node );

	void home( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
	void home( double );
	void computeHomePosition(const osg::Camera *camera = NULL, bool useBoundingBox = false);

	virtual bool handleFrame( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );

	void flyTo(const osg::Vec3d& newEye, const osg::Vec3d& newCenter, const osg::Vec3d& newUp, double time);
private:
	osg::Vec3d _oldCenter;
	osg::Vec3d _oldEye;
	osg::Vec3d _oldUp;
	osg::Vec3d _vtCenter;
	osg::Vec3d _vtEye;
	osg::Vec3d _vtUp;
	double _totalTime;
	double _elapseTime;
	bool _flyTo;
};

#endif