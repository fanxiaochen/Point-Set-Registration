/**
 * @file
 * @author  Xiaochen Fan <fan.daybreak@gmail.com>
 * @version 1.0
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * http://www.gnu.org/copyleft/gpl.html
 *
 * @section DESCRIPTION
 *
 * 
 */

#ifndef RENDER_THREAD_HPP
#define RENDER_THREAD_HPP

#include <OpenThreads/Thread>
#include <osg/Group>
#include <osg/Geometry>
#include <osg/Point>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>

namespace cpd
{
    template <typename T, int D>
    class RenderThread : public OpenThreads::Thread
    {
    public:
        static RenderThread* instance()
        {
            static RenderThread render_thread;
            return &render_thread;
        }

        virtual int cancel();
        virtual void run();

        void updateModel(const TMatrix& model);
        void updateData(const TMatrix& data);

        inline void getModel(){ return _model; }
        inline void getData(){ return _data; }

        void updateModelGeometry();
        void updateDataGeometry();

    protected:
        OpenThreads::Mutex _mutex;

        TMatrix _model;
        TMatrix _data;

        osg::ref_ptr<osg::Geode> _m_node;
        osg::ref_ptr<osg::Geode> _d_node;
        osg::ref_ptr<osg::Group> _scene;

        bool _done;
    };

    template <typename T, int D>
    class ModelCallback : public osg::NodeCallback 
    {
    public:
        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            osg::ref_ptr<osg::Geode> m_node = 
                dynamic_cast<osg::Geode*>(node);

            if(m_node)
                RenderThread<T, D>::instance()->updateModelGeometry();
          
            traverse(node, nv); 
        }
    };

    template <typename T, int D>
    class DataCallback : public osg::NodeCallback 
    {
    public:
        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            osg::ref_ptr<osg::Geode> d_node = 
                dynamic_cast<osg::Geode*>(node);

            if(d_node)
                RenderThread<T, D>::instance()->updateDataGeometry();

            traverse(node, nv); 
        }
    };
}


namespace cpd
{
    template <typename T, int D>
    void RenderThread<T, D>::run()
    {
        std::cout << "Rendering Starts!" << std::endl;
        _m_node = new osg::Geode;
        _d_node = new osg::Geode;
        _scene = new osg::Group;
        _scene->addChild(_m_node);
        _scene->addChild(_d_node);
        updateModelGeometry();
        updateDataGeometry();

        _m_node->setUpdateCallback(new ModelCallback<T, D>);
        _d_node->setUpdateCallback(new DataCallback<T, D>);

        osgViewer::Viewer viewer;
        viewer.getCamera()->setClearColor(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
        viewer.setSceneData(_scene);

        viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

        viewer.run();

        return;
    }

    template <typename T, int D>
    int RenderThread<T, D>::cancel()
    {
        while(isRunning()) YieldCurrentThread();
        return 0;
    }

    template <typename T, int D>
    void RenderThread<T, D>::updateModel(const TMatrix& model)
    {
        _model = model;
    }

    template <typename T, int D>
    void RenderThread<T, D>::updateData(const TMatrix& data)
    {
        _data = data;
    }

    template <typename T, int D>
    void RenderThread<T, D>::updateModelGeometry()
    {
        if (D > 3)
        {
            std::cerr << "can't visualize data higher than three dimension!" << std::endl;
            return;
        }

        _m_node->removeDrawables(0);

        osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
        osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;

        for(size_t i = 0, i_end = _model.rows(); i < i_end; i++)   
        {
            osg::Vec3 point;
            const TVector& m_vec = _model.row(i);
            
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
            
        geometry->setVertexArray(points);

        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
        colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 0.0f));
        geometry->setColorArray(colors.get());
        geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

        osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
        normals->push_back(osg::Vec3(0.0f, -1.0f, 0.0f));
        geometry->setNormalArray(normals.get());
        geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);

        geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points->size()));
        geometry->getOrCreateStateSet()->setAttribute(new osg::Point(5.0f));
        _m_node->addDrawable(geometry.get());

        return;
    }

    template <typename T, int D>
    void RenderThread<T, D>::updateDataGeometry()
    {
        if (D > 3)
        {
            std::cerr << "can't visualize data higher than three dimension!" << std::endl;
            return;
        }

        _d_node->removeDrawables(0);

        osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
        osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;

        for(size_t i = 0, i_end = _data.rows(); i < i_end; i++)   
        {
            osg::Vec3 point;
            const TVector& d_vec = _data.row(i);
            
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
            
        geometry->setVertexArray(points);

        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
        colors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 0.0f));
        geometry->setColorArray(colors.get());
        geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

        osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
        normals->push_back(osg::Vec3(0.0f, -1.0f, 0.0f));
        geometry->setNormalArray(normals.get());
        geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);

        geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points->size()));
        geometry->getOrCreateStateSet()->setAttribute(new osg::Point(5.0f));
        _d_node->addDrawable(geometry.get());

        return;
    }
}


#endif