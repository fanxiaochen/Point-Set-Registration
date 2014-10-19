#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP
#include <osg/Group>
#include <osg/Geometry>
#include <osg/Point>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>

#include <base/matrix.hpp>

namespace cpd
{
	template <typename T, int D>
	class Visualizer
	{
	public:
		Visualizer();
		~Visualizer();

		void updateModel(const TMatrixD& model);
		void updateData(const TMatrixD& data);

		int show();

	private:
		TMatrixD _model;
		TMatrixD _data;

		osg::ref_ptr<osg::Geometry> _m_geo;
		osg::ref_ptr<osg::Geometry> _d_geo;

		osg::ref_ptr<osg::Geode> _scene;
		osgViewer::Viewer _viewer;
	};
}

namespace cpd
{
	template <typename T, int D>
	Visualizer<T, D>::Visualizer()
		:_m_geo(new osg::Geometry),
		_d_geo(new osg::Geometry),
		_scene(new osg::Geode)
	{}

	template <typename T, int D>
	Visualizer<T, D>::~Visualizer(){}

	template <typename T, int D>
	void Visualizer<T, D>::updateModel(const TMatrixD& model)
	{
		if (D > 3)
			std::cout << "can't visualize data higher than three dimension!" << std::endl;

		if (!_m_geo->empty())
		{
			_scene->removeDrawable(_m_geo);
			_m_geo->getPrimitiveSetList().clear();
		}

		osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;

		for(size_t i = 0, i_end = model.rows(); i < i_end; i++)   
		{
			osg::Vec3 point;
			const TVector& m_vec = model.row(i);
			
			if (D == 3)
			{
				point.x() = m_vec(0);
				point.y() = m_vec(1);
				point.z() = m_vec(2);
			}
				
			else
			{
				point.x() = m_vec(0);
				point.y() = 0;
				point.z() = m_vec(1);
			}
				

			points->push_back(point);
		}
			
		_m_geo->setVertexArray(points);

		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
		colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 0.0f));
		_m_geo->setColorArray(colors.get());
		_m_geo->setColorBinding(osg::Geometry::BIND_OVERALL);

		osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
		normals->push_back(osg::Vec3(0.0f, -1.0f, 0.0f));
		_m_geo->setNormalArray(normals.get());
		_m_geo->setNormalBinding(osg::Geometry::BIND_OVERALL);

		_m_geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points->size()));
		_m_geo->getOrCreateStateSet()->setAttribute(new osg::Point(5.0f));
		_scene->addDrawable(_m_geo.get());

		return;
	}

	template <typename T, int D>
	void Visualizer<T, D>::updateData(const TMatrixD& data)
	{
		if (D > 3)
			std::cout << "can't visualize data higher than three dimension!" << std::endl;

		if (!_d_geo->empty())
		{
			_scene->removeDrawable(_d_geo);
			_d_geo->getPrimitiveSetList().clear();
		}

		osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;

		for(size_t i = 0, i_end = data.rows(); i < i_end; i++)   
		{
			osg::Vec3 point;
			const TVector& d_vec = data.row(i);
			if (D == 3)
			{
				point.x() = d_vec(0);
				point.y() = d_vec(1);
				point.z() = d_vec(2);
			}
				
			else
			{
				point.x() = d_vec(0);
				point.y() = 0;
				point.z() = d_vec(1);
			}

			points->push_back(point);
		}
			

		_d_geo->setVertexArray(points);

		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
		colors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 0.0f));
		_d_geo->setColorArray(colors.get());
		_d_geo->setColorBinding(osg::Geometry::BIND_OVERALL);

		osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
		normals->push_back(osg::Vec3(0.0f, -1.0f, 0.0f));
		_d_geo->setNormalArray(normals.get());
		_d_geo->setNormalBinding(osg::Geometry::BIND_OVERALL);

		_d_geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points->size()));
		_d_geo->getOrCreateStateSet()->setAttribute(new osg::Point(5.0f));
		_scene->addDrawable(_d_geo.get());

		return;
	}

	template <typename T, int D>
	int Visualizer<T, D>::show()
	{
		_viewer.getCamera()->setClearColor(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
		_viewer.setSceneData(_scene);

		_viewer.addEventHandler(new osgGA::StateSetManipulator(_viewer.getCamera()->getOrCreateStateSet()));
		_viewer.addEventHandler(new osgViewer::StatsHandler);
		_viewer.addEventHandler(new osgViewer::WindowSizeHandler);

		return _viewer.run();
	}
}






#endif