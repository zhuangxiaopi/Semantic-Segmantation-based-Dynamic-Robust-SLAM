/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

#define PEOPLE_LABEL 31
#define EXPANSION 30

namespace ORB_SLAM2
{
MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):
        mpMap(pMap),
        m_octree(NULL),
        m_maxRange(-1.0),
        m_useHeightMap(true),
        m_res(0.05),
        m_colorFactor(0.8),
        m_treeDepth(0),
        m_maxTreeDepth(0)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
    m_res            = fSettings["octoMap.res"];// octomap图精度


    cam_width = fSettings["Camera.width"];
    cam_height = fSettings["Camera.height"];
    cam_fx = fSettings["Camera.fx"];
    cam_fy = fSettings["Camera.fy"];
    cam_cx = fSettings["Camera.cx"];
    cam_cy = fSettings["Camera.cy"];

    m_octree = new octomap::ColorOcTree(m_res);// octomap图精度
    // initialize octomap
    m_octree->setClampingThresMin(0.12); // 这些参数都可以传进来===
    m_octree->setClampingThresMax(0.97);
    m_octree->setProbHit(0.7);
    m_octree->setProbMiss(0.4);

    m_treeDepth = m_octree->getTreeDepth();
    m_maxTreeDepth = m_treeDepth;
    bIsLocalization = false;
}

MapDrawer::~MapDrawer()
{
    delete m_octree;
}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

void MapDrawer::DrawGrid()
{
    glBegin(GL_LINES);// 画线 ======
    glLineWidth(1);       // 线宽======

    glColor3f(0.5,0.5,0.5); //gray  灰色线条
    int size =10;
    for(int i = -size; i <= size ; i++){
// xz 平面 垂直x轴直线  x轴上-10,-9,...,0,1,2,...,10 21条线，z方向范围， -10～10
        glVertex3f(i,0.6,  size);
        glVertex3f(i, 0.6, -size);

// xz 平面 垂直z轴直线z轴上 -10,-9,...,0,1,2,...,10  21条线，x方向范围， -10～10
        glVertex3f( size, 0.6, i);
        glVertex3f(-size, 0.6, i);
    }
    glEnd();
}

// 在胖果林中显示 octomap=========
void MapDrawer::DrawOctoMap() {
        PointCloud cld;
        vector<KeyFrame *> vKFs = mpMap->GetAllKeyFrames();//  获取所有关键帧
        int N = vKFs.size();// 当前关键帧数量
        cout << "--GeneratePointCloud生成当前帧点云--" << endl;
        if (bIsLocalization == false) {
            if (N == 0) {
                m_octree->clear();// 八叉树地图清空
                cout << "Keyframes miss!" << endl;
                lastKeyframeSize = 0;
            }
            if (N > 1) {
                for (size_t i = lastKeyframeSize; i < (unsigned int) N - 1; i++) {
                    GeneratePointCloud(vKFs[i]);
                }
                lastKeyframeSize = N - 1;
            }
        }
            // 带有颜色的
            octomap::ColorOcTree::tree_iterator it = m_octree->begin_tree();
            octomap::ColorOcTree::tree_iterator end = m_octree->end_tree();
            int counter = 0;// 计数
            double occ_thresh = 0.8; // 概率阈值 原来 0.9  越大，显示的octomap格子越少
            int level = 16; // 八叉树地图 深度???
//    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);// 颜色 + 透明度
            glDisable(GL_LIGHTING);
            glEnable(GL_BLEND);
            cout << "--DRAW OCTOMAP BEGIN--" << endl;
            ////DRAW OCTOMAP BEGIN//////
            for (; it != end; ++counter, ++it) {
                if (level != it.getDepth()) {
                    continue;
                }
                double occ = it->getOccupancy();//占有概率=================
                if (occ < occ_thresh) // 占有概率较低====不显示
                {
                    continue;
                }

                // std::cout<< occ << std::endl;
                double minX, minY, minZ, maxX, maxY, maxZ;
                m_octree->getMetricMin(minX, minY, minZ);
                m_octree->getMetricMax(maxX, maxY, maxZ);
                float halfsize = it.getSize() / 2.0;// 半尺寸
                float x = it.getX();
                float y = it.getY();
                float z = it.getZ();
//                cout<<"it->getColor(): "<<it->getColor()<<endl;
                octomap::ColorOcTreeNode::Color its = it->getColor();
            glBegin(GL_TRIANGLES); // 三角形??
            //Front
            glColor3d(its.r,its.g,its.b);// 显示颜色=====
            glVertex3f(x - halfsize, y - halfsize, z - halfsize);// - - - 1
            glVertex3f(x - halfsize, y + halfsize, z - halfsize);// - + - 2
            glVertex3f(x + halfsize, y + halfsize, z - halfsize);// + + -3

            glVertex3f(x - halfsize, y - halfsize, z - halfsize); // - - -
            glVertex3f(x + halfsize, y + halfsize, z - halfsize); // + + -
            glVertex3f(x + halfsize, y - halfsize, z - halfsize); // + - -4

            //Back
            glVertex3f(x - halfsize, y - halfsize, z + halfsize); // - - + 1
            glVertex3f(x + halfsize, y - halfsize, z + halfsize); // + - + 2
            glVertex3f(x + halfsize, y + halfsize, z + halfsize); // + + + 3

            glVertex3f(x - halfsize, y - halfsize, z + halfsize); // - - +
            glVertex3f(x + halfsize, y + halfsize, z + halfsize); // + + +
            glVertex3f(x - halfsize, y + halfsize, z + halfsize); // - + + 4
            //Left
            glVertex3f(x - halfsize, y - halfsize, z - halfsize); // - - - 1
            glVertex3f(x - halfsize, y - halfsize, z + halfsize); // - - + 2
            glVertex3f(x - halfsize, y + halfsize, z + halfsize); // - + + 3

            glVertex3f(x - halfsize, y - halfsize, z - halfsize); // - - -
            glVertex3f(x - halfsize, y + halfsize, z + halfsize); // - + +
            glVertex3f(x - halfsize, y + halfsize, z - halfsize); // - + - 4

            //Right
            glVertex3f(x + halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);

            glVertex3f(x + halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);
            glVertex3f(x + halfsize, y - halfsize, z + halfsize);

            //top
            glVertex3f(x - halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y - halfsize, z + halfsize);

            glVertex3f(x - halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y - halfsize, z + halfsize);
            glVertex3f(x - halfsize, y - halfsize, z + halfsize);

            //bottom
            glVertex3f(x - halfsize, y + halfsize, z - halfsize);
            glVertex3f(x - halfsize, y + halfsize, z + halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);

            glVertex3f(x - halfsize, y + halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);
            glVertex3f(x + halfsize, y + halfsize, z - halfsize);
            glEnd();

            glBegin(GL_LINES); // 线段=======
            glColor3f(0, 0, 0);
            //
            glVertex3f(x - halfsize, y - halfsize, z - halfsize);// - - - 1
            glVertex3f(x - halfsize, y + halfsize, z - halfsize);

            glVertex3f(x - halfsize, y + halfsize, z - halfsize);// - + - 2
            glVertex3f(x + halfsize, y + halfsize, z - halfsize);// + + -3

            glVertex3f(x + halfsize, y + halfsize, z - halfsize);// + + -3
            glVertex3f(x + halfsize, y - halfsize, z - halfsize); // + - -4

            glVertex3f(x + halfsize, y - halfsize, z - halfsize); // + - -4
            glVertex3f(x - halfsize, y - halfsize, z - halfsize);// - - - 1


            // back

            glVertex3f(x - halfsize, y - halfsize, z + halfsize); // - - + 1
            glVertex3f(x + halfsize, y - halfsize, z + halfsize); // + - + 2

            glVertex3f(x + halfsize, y - halfsize, z + halfsize); // + - + 2
            glVertex3f(x + halfsize, y + halfsize, z + halfsize); // + + + 3

            glVertex3f(x + halfsize, y + halfsize, z + halfsize); // + + + 3
            glVertex3f(x - halfsize, y + halfsize, z + halfsize); // - + + 4

            glVertex3f(x - halfsize, y + halfsize, z + halfsize); // - + + 4
            glVertex3f(x - halfsize, y - halfsize, z + halfsize); // - - + 1

            // top
            glVertex3f(x + halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y - halfsize, z + halfsize);

            glVertex3f(x - halfsize, y - halfsize, z + halfsize);
            glVertex3f(x - halfsize, y - halfsize, z - halfsize);

            // bottom

            glVertex3f(x - halfsize, y + halfsize, z + halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);

            glVertex3f(x - halfsize, y + halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z - halfsize);
            glEnd();
        }
}
// 生成当前帧的点云
void MapDrawer::GeneratePointCloud(KeyFrame *kf)
{
    PointCloud::Ptr tmp( new PointCloud() );
    // Point cloud is null ptr
    for ( int m=0; m<kf->mImDep.rows; m+=1)
    {
        for ( int n=0; n<kf->mImDep.cols; n+=1)
        {
            float d = kf->mImDep.ptr<float>(m)[n];
            if (d < 0.01 || d > 4)
                continue;

            if((int)kf->mImLabel.ptr<uchar>(m)[n] != PEOPLE_LABEL && (int)kf->mImFmask.ptr<uchar>(m)[n] != 0)
            {
                    //语义分割膨胀，剔除Label边缘的特征点，进一步减小动态目标的影响
                if((int)kf->mImLabel.ptr<uchar>(m+EXPANSION)[n+EXPANSION] != PEOPLE_LABEL &&
                   (int)kf->mImLabel.ptr<uchar>(m+EXPANSION)[n-EXPANSION] != PEOPLE_LABEL &&
                   (int)kf->mImLabel.ptr<uchar>(m+EXPANSION)[n+EXPANSION] != PEOPLE_LABEL &&
                   (int)kf->mImLabel.ptr<uchar>(m+EXPANSION)[n+EXPANSION] != PEOPLE_LABEL &&
                    //光流检测膨胀，剔除Label边缘的特征点，进一步减小动态目标的影响
                   (int)kf->mImFmask.ptr<uchar>(m+EXPANSION)[n+EXPANSION] != 0 &&
                   (int)kf->mImFmask.ptr<uchar>(m+EXPANSION)[n-EXPANSION] != 0 &&
                   (int)kf->mImFmask.ptr<uchar>(m+EXPANSION)[n+EXPANSION] != 0 &&
                   (int)kf->mImFmask.ptr<uchar>(m+EXPANSION)[n+EXPANSION] != 0)
                {
                    PointT p;
                    p.z = d;
                    p.x = ( n - cam_cx) * p.z / cam_fx;
                    p.y = ( m - cam_cy) * p.z / cam_fy;

                    // Deal with color
                    if((int)kf->mImLabel.ptr<uchar>(m)[n]==0)
                    {
                        p.b = kf->mImRGB.ptr<uchar>(m)[n*3];
                        p.g = kf->mImRGB.ptr<uchar>(m)[n*3+1];
                        p.r = kf->mImRGB.ptr<uchar>(m)[n*3+2];
                    }
                    else
                    {
                        p.b = kf->mImColor.ptr<uchar>(m)[n*3];
                        p.g = kf->mImColor.ptr<uchar>(m)[n*3+1];
                        p.r = kf->mImColor.ptr<uchar>(m)[n*3+2];
//                        p.b = kf->mImRGB.ptr<uchar>(m)[n*3];
//                        p.g = kf->mImRGB.ptr<uchar>(m)[n*3+1];
//                        p.r = kf->mImRGB.ptr<uchar>(m)[n*3+2];
//                        cout<<"p.b: "<<p.b<<" , p.g: "<<p.g<<" , p.r: "<<p.r<<endl;
                    }
                    tmp->points.push_back(p);
                }

            }
        }
    }

//    // 体素格滤波======
    voxel.setInputCloud(tmp);
    voxel.setLeafSize(0.01,0.01, 0.01);// 体素格子 尺寸
    voxel.filter(*tmp);

    // 转换到世界坐标下====
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(kf->GetPose());
    PointCloud cloud;
    pcl::transformPointCloud( *tmp, cloud, T.inverse().matrix());

    cout<<"Generate point cloud for kf "<<kf->mnId<<", size="<<cloud.size()<<endl;
    octomap::point3d sensorOrigin = octomap::point3d( T(0,3), T(1,3), T(2,3));// 点云原点
//将新点云 插入到 octomap地图中
    InsertScan(sensorOrigin, cloud);
}

void MapDrawer::InsertScan(octomap::point3d sensorOrigin,  // 点云原点
                           PointCloud &cloud)//点云
{
    if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin) ||
        !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax)) {
        printf("coulde not generate key for origin\n");
    }

    octomap::KeySet free_cells, occupied_cells;// 空闲格子，占有格子

// 每一个点云=======================
    for (auto p:cloud.points) {
        octomap::point3d point(p.x, p.y, p.z);
        if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)) {
            free_cells.insert(m_keyRay.begin(), m_keyRay.end()); // 地面为空闲格子
            m_octree->averageNodeColor(p.x, p.y, p.z, p.r, p.g, p.b);//颜色
//            cout<<"p.b: "<<p.b<<" , p.g: "<<p.g<<" , p.r: "<<p.r<<endl;
        }

        octomap::OcTreeKey key;
        if (m_octree->coordToKeyChecked(point, key)) {
            occupied_cells.insert(key); // 占有格子
            updateMinKey(key, m_updateBBXMin);
            updateMaxKey(key, m_updateBBXMax);
            m_octree->averageNodeColor(p.x, p.y, p.z, p.r,p.g, p.b);
//            cout<<"p.b: "<<p.b<<" , p.g: "<<p.g<<" , p.r: "<<p.r<<endl;
        }
    }
    // 空闲格子====
    for(octomap::KeySet::iterator it = free_cells.begin(),
                end= free_cells.end();
        it!=end; ++it)
    {
        if(occupied_cells.find(*it) == occupied_cells.end())// 占有格子未找到
        {
            m_octree->updateNode(*it, false);// 空闲格子
        }
    }
// 占有格子====
    for(octomap::KeySet::iterator it = occupied_cells.begin(),
                end = occupied_cells.end();
        it!=end; ++it)
    {
        m_octree->updateNode(*it, true);// 占有格子
    }

    m_octree->prune();
}

void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

// 保存地图为octomap=====
void MapDrawer::SaveOctoMap(const char *filename)
{
    std::ofstream outfile(filename, std::ios_base::out | std::ios_base::binary);
    if (outfile.is_open())
    {
        m_octree->write(outfile);
        outfile.close();
    }
}

} //namespace ORB_SLAM