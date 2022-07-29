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
    // ������Ⱦ����
    void initializeGL();
    // ���ƴ���
    void paintGL();
    // ��Ӧ���ڵĴ�С�仯
    void resizeGL(int width, int height);

private:
    // ������Ⱦ
    void renderScene();
    // ������Ⱦ-����ͼ��
    void renderBasicShape();
    // ������Ⱦ-����������
    void renderTextureCube();
    // ������Ⱦ-Բ��������
    void renderTextureCylinder();
    // ����������
    void drawCube();
    // ����Բ��
    void drawCircle(float);
    // ����Բ����
    void drawCylinder(float);
    // ��������
    void loadGLTextures();

    void drawLine();
    void setRotateAndTranslation();
    void renderContinuumShape();
    void drawContinuumSpacer();
    void forwardKinematics();
private:
    // �洢����
    GLuint texture[2];

    float spacer_ring_radius; //���Ƭ�뾶
    float longitudinal_resolution;
    float transverse_resolution;
    float spacer_ply;       //���Ƭ���

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