#pragma once
#ifndef CONTINUUM_SHOW_H_
#define CONTINUUM_SHOW_H_

#include <QOpenGLWidget>
#include "configration_space.h"
#include "tool_component.h"

class ContinuumShow : public QOpenGLWidget
{
    Q_OBJECT

public:
    ContinuumShow(QWidget* parent = 0);
    ~ContinuumShow();

protected:
    // 设置渲染环境
    void initializeGL();
    // 绘制窗口
    void paintGL();
    // 响应窗口的大小变化
    void resizeGL(int width, int height);

private:
    // 场景渲染
    void renderScene();
    // 场景渲染-基本图形
    void renderBasicShape();
    // 场景渲染-立方体纹理
    void renderTextureCube();
    // 场景渲染-圆柱体纹理
    void renderTextureCylinder();
    // 绘制立方体
    void drawCube();
    // 绘制圆形
    void drawCircle(float);
    // 绘制圆柱体
    void drawCylinder(float);
    // 加载纹理
    void loadGLTextures();

    void drawLine();
    void setRotateAndTranslation();
    void renderContinuumShape();
    void drawContinuumSpacer();
    void forwardKinematics();
private:
    // 存储纹理
    GLuint texture[2];

    float spacer_ring_radius; //间隔片半径
    float longitudinal_resolution;
    float transverse_resolution;
    float spacer_ply;       //间隔片厚度

public:
    int mos_x, mos_y;
    bool is_reset;
    bool left_clicked, right_clicked, is_range;
    bool w_clicked, a_clicked , s_clicked, d_clicked, q_clicked, e_clicked;
    float rot_angle_x, rot_angle_y, trans_x, trans_y,trans_z, m_speed;

    bool _1st_rigid_is_bend;
    ConfigSpace cfg_sp;
    Segment segments;
    PhisicalPara pp;
    StructurePara sp; 
};
#endif // CONTINUUM_SHOW_H_