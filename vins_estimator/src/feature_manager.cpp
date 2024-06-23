#include "feature_manager.h"


Line2D::Line2D(const Eigen::Vector4d &Vec)
{
    PtrStart = Vector2d(Vec[0], Vec[1]);
    PtrEnd = Vector2d(Vec[2], Vec[3]);
    LineVec = PtrEnd - PtrStart;
    Length = LineVec.norm();
    Direction = LineVec / Length;
    A = PtrEnd[1] - PtrStart[1];
    B = PtrStart[0] - PtrEnd[0];
    C = PtrEnd[0] * PtrStart[1] - PtrStart[0] * PtrEnd[1];
    A2B2 = sqrt(A * A + B * B);
}

Line2D::Line2D(const Eigen::Vector4d &Vec, const Eigen::Matrix3d &K)
{
    PtrStart = Vector2d(Vec[0], Vec[1]);
    PtrEnd = Vector2d(Vec[2], Vec[3]);
    LineVec = PtrEnd - PtrStart;
    Length = LineVec.norm();
    Direction = LineVec / Length;
    PtrMid = (PtrStart + PtrEnd) / 2;
    hPtrStart = Vector3d(PtrStart[0], PtrStart[1], 1);
    hPtrEnd = Vector3d(PtrEnd[0], PtrEnd[1], 1);

    A = PtrEnd[1] - PtrStart[1];
    B = PtrStart[0] - PtrEnd[0];
    C = PtrEnd[0] * PtrStart[1] - PtrStart[0] * PtrEnd[1];
    A2B2 = sqrt(A * A + B * B);
    Gradient = atan2(PtrStart[1] - PtrEnd[1], PtrStart[0] - PtrEnd[0]) / PI * 180;
    float fx = K(0, 0);
    float fy = K(1, 1);
    float cx = K(0, 2);
    float cy = K(1, 2);

    //ROS_INFO("the fx, fy, cx, cy = %f, %f, %f, %f", fx, fy, cx, cy);

    LineObsN[0] = (PtrStart[0] - cx) / fx;
    LineObsN[1] = (PtrStart[1] - cy) / fy;
    LineObsN[2] = (PtrEnd[0] - cx) / fx;
    LineObsN[3] = (PtrEnd[1] - cy) / fy;
}

Eigen::Vector2d Line2D::Point2Flined(const Eigen::Vector2d &minP)
{
    Vector2d tmpStart = minP - PtrStart;
    double d1 = tmpStart.norm();
    Vector2d tmpEnd = minP - PtrEnd;
    double d2 = tmpEnd.norm();

    double A_, B_, C_;
    A_ = B;
    B_ = -A;
    C_ = -1 * (A_ * minP[0] + B_ * minP[1]);
    Matrix2d Cof;
    Cof << A, B, A_, B_;
    Vector2d intersection = Cof.inverse() * Vector2d(-C, -C_);
    if ((intersection.x()-PtrStart.x())*(intersection.x()-PtrEnd.x())>=0)
    {
        if (d1 < d2)
            return PtrStart;
        else
            return PtrEnd;
    }
    else
    {
        return intersection;
    }
}

Eigen::Vector3d Line2D::ComputeCircleNormal(const Eigen::Matrix3d &K)
{
    Vector3d P1 = K.inverse() * hPtrStart;
    Vector3d P2 = K.inverse() * hPtrEnd;
    MatrixXd A(3, 4);
    A << 0, 0, 0, 1,
            P1.transpose(), 1,
            P2.transpose(), 1;
    EigenSolver<MatrixXd> ES(A.transpose() * A);
    Vector4d V = ES.eigenvectors().col(2).real();
    if (ES.eigenvalues()[2].real() > ES.eigenvalues()[3].real())
        V = ES.eigenvectors().col(3).real();

    Vector3d normal(V[0], V[1], V[2]);
    return normal.normalized();
}

Line3D::Line3D(const Vector6d &Vec)
{
    PtrStart = Vector3d(Vec[0], Vec[1], Vec[2]);
    PtrEnd = Vector3d(Vec[3], Vec[4], Vec[5]);
    Vector3d LineVec = PtrEnd - PtrStart;
    Length = LineVec.norm();
    Direction = LineVec / Length;
}

Line3D::Line3D(const Eigen::Vector3d &ptrStart, const Eigen::Vector3d &ptrEnd)
{
    PtrStart = ptrStart;
    PtrEnd = ptrEnd;
    Vector3d LineVec = PtrEnd - PtrStart;
    Length = LineVec.norm();
    Direction = LineVec / Length;

}

Line3D Line3D::Transform3D(const Eigen::Matrix3d &R, const Eigen::Vector3d &t)
{
    Vector3d tempStart = R * PtrStart + t;
    Vector3d tempEnd = R * PtrEnd + t;
    Line3D TransLine(tempStart, tempEnd);
    return TransLine;
}

Line2D Line3D::Project3D(const Eigen::Matrix3d &K)
{
    Vector3d tempS = K * PtrStart;
    double p1 = tempS[0] / tempS[2];
    double p2 = tempS[1] / tempS[2];
    Vector3d tempE = K * PtrEnd;
    double p3 = tempE[0] / tempE[2];
    double p4 = tempE[1] / tempE[2];
    Line2D ProLine(Vector4d(p1, p2, p3, p4), K);
    return ProLine;
}

PairsMatch::PairsMatch(int idx, Line3D &lineB, Line3D &lineC, Line2D &projectedL, Line2D &detectedL)
{
    index = idx;
    line3dBodyP = lineB;
    line3dcamP = lineC;
    projectedlineP = projectedL;
    detectedlineP = detectedL;
    weight = 0.0;
}

bool Compare(PairsMatch pair1, PairsMatch pair2)
{
    return (pair1.criteria[0] < pair2.criteria[0]);   // from small to big
}

int lineFeaturePerId::endFrame()
{
    return start_frame + linefeature_per_frame.size() - 1;
}



int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature)
    {

        it.used_num = it.feature_per_frame.size();

        if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2)
        {
            cnt++;
        }
    }
    return cnt;
}

int FeatureManager::getLineFeatureCount()
{
    int cnt = 0;
    for (auto &it : linefeature)
    {
        it.used_num = it.linefeature_per_frame.size();
        if (it.used_num >= 2 && it.used_num < WINDOW_SIZE - 2)
        {
            cnt++;
        }
    }
    return cnt;
}

bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td)
{
    ROS_DEBUG("input feature: %d", (int)image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });

        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
        }
    }

    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

bool FeatureManager::addFeaturesCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
                                              const vector<pair<int, Eigen::Vector4d>> &lines, double td)
{
    ROS_DEBUG("input feature: %d", (int)image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    ROS_DEBUG("num of line feature: %d", getLineFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });
        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
        }
    }

    for (auto &id_line : lines)
    {
        lineFeaturePerFrame lf_per_fra(id_line.second);
        int feature_id = id_line.first;
        auto it = find_if(linefeature.begin(), linefeature.end(), [feature_id](const lineFeaturePerId &it)
        {
            return it.feature_id == feature_id;
        });
        if (it == linefeature.end())
        {
            linefeature.push_back(lineFeaturePerId(feature_id, frame_count));
            linefeature.back().linefeature_per_frame.push_back(lf_per_fra);
        }

        else if (it->feature_id == feature_id)
        {
            it->linefeature_per_frame.push_back(lf_per_fra);
            it->all_obs_cnt++;
        }

    }

    if (frame_count < 2 || last_track_num < 20)
        return true;
    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
        it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }
    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }

}


void FeatureManager::debugShow()
{
    ROS_DEBUG("debug show");
    for (auto &it : feature)
    {
        ROS_ASSERT(it.feature_per_frame.size() != 0);
        ROS_ASSERT(it.start_frame >= 0);
        ROS_ASSERT(it.used_num >= 0);

        ROS_DEBUG("%d,%d,%d ", it.feature_id, it.used_num, it.start_frame);
        int sum = 0;
        for (auto &j : it.feature_per_frame)
        {
            ROS_DEBUG("%d,", int(j.is_used));
            sum += j.is_used;
            printf("(%lf,%lf) ",j.point(0), j.point(1));
        }
        ROS_ASSERT(it.used_num == sum);
    }
}

vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}

void FeatureManager::clearDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
    }
}

VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}

void FeatureManager::triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (it_per_id.estimated_depth > 0)
            continue;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        ROS_ASSERT(NUM_OF_CAM == 1);
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }

    }
}

void FeatureManager::removeLineOutlier()
{
    for (auto &it_per_id : linefeature)
    {
       // int start_idx = it_per_id.start_frame;
        int obvers_time = it_per_id.linefeature_per_frame.size();
        if (obvers_time < 1)  // tracking time < 2, but still is likely to be tracked continue.
        {
         //   cout << "the observation times of the same line < 2" << endl;
         //   it_per_id.credible_matching = false;   // ??????
         //   continue;
        }
        else
        {
            Line3D start_line = it_per_id.linefeature_per_frame.begin()->lineWorld;
            int count = 0;
            for (auto &it_per_frame : it_per_id.linefeature_per_frame)
            {
                Line3D second_line = it_per_frame.lineWorld;
                float diff_ = lineDiff(start_line, second_line);
                it_per_id.statistics_error_per_id.push_back(diff_);
                if (diff_ > 0.1)  // unit = meter
                {
                    count++;
                    it_per_frame.credible_line = false;
                    cout << "this is a incredible_line" << endl;
                }
                else
                    it_per_frame.credible_line = true;
            }
            if ((count / obvers_time) >= 0.5)
            {
                it_per_id.credible_matching = false;
                cout << "this is a incredible_matching" << endl;
            }
            else
                it_per_id.credible_matching = true;
        }

    }
}

float FeatureManager::lineDiff(Line3D L1, Line3D L2)
{
    Eigen::Vector3d D = L1.LineVec - L2.LineVec;
    float diff = D.norm();
    return diff;
}

void FeatureManager::removeOutlier()
{
    ROS_BREAK();
    int i = -1;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        i += it->used_num != 0;
        if (it->used_num != 0 && it->is_outlier == true)
        {
            feature.erase(it);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }

    // for the line feature
    for (auto it = linefeature.begin(), it_next = linefeature.begin();
         it != linefeature.end() ; it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
        {
            it->start_frame--;
        }
        else
        {
            it->linefeature_per_frame.erase(it->linefeature_per_frame.begin());
            if (it->linefeature_per_frame.size() == 0)
                linefeature.erase(it);
        }
    }

}

void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }

    // for the line feature
    for (auto it = linefeature.begin(), it_next = linefeature.begin();
              it != linefeature.end() ; it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
        {
            it->start_frame--;
        }
        else
        {
            it->linefeature_per_frame.erase(it->linefeature_per_frame.begin());
            if (it->linefeature_per_frame.size() == 0)
                linefeature.erase(it);
        }
    }

}

void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }

    // for the line feature
    for (auto it = linefeature.begin(), it_next = linefeature.begin();
              it != linefeature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count -1)
                continue;
            it->linefeature_per_frame.erase(it->linefeature_per_frame.begin() + j);
            if (it->linefeature_per_frame.size() == 0)
                linefeature.erase(it);
        }
    }
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}