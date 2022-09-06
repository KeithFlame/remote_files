#pragma once
#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QOpenGLWidget>

class GLWidget : public QOpenGLWidget
{
    Q_OBJECT

public:
    GLWidget(QOpenGLWidget* parent = 0);
    ~GLWidget();

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
    void drawCircle();
    // ����Բ����
    void drawCylinder();
    // ����Բ׶��
    void drawCone();
    // ����������
    void drawTetrahedron();
    // ��������
    void drawSphere(GLfloat xx = 0.0,
        GLfloat yy = 0.0,
        GLfloat zz = 0.0,
        GLfloat radius = 1.0,
        GLfloat M = 100.0,
        GLfloat N = 100.0);
    // ����Բ��
    void DrawTorus(double Radius = 1,
        double TubeRadius = 0.2,
        int Sides = 20,
        int Rings = 30);
    // ��������
    void loadGLTextures();


private:
    // ��ת�Ƕ�
    GLfloat  xRot;
    GLfloat  yRot;
    GLfloat  zRot;
    // �洢����
    GLuint texture[2];

};
#endif // GLWIDGET_H