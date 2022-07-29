#include "continuum_show.h"
#include <qmath.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define PI          3.1416
#define ROT_DELTA   0.5f

void qgluPerspective(GLdouble dfov, GLdouble aspect, GLdouble zNear, GLdouble zFar)
{
    const GLdouble ymax = (zNear * tan(qDegreesToRadians(dfov) / 2.0))/qSqrt(1+qPow(aspect,2));

    const GLdouble ymin = -ymax;
    const GLdouble xmin = ymin * aspect;
    const GLdouble xmax = ymax * aspect;
    glFrustum(xmin, xmax, ymin, ymax, zNear, zFar);
}

ContinuumShow::ContinuumShow(QWidget* parent)
    : QOpenGLWidget(parent)
    , mos_x(0)
    , mos_y(0)
    , rot_angle_x(0.f)
    , rot_angle_y(0.f)
    , trans_x(0.f)
    , trans_y(0.f)
    , trans_z(-12.f)
    , left_clicked(false)
    , right_clicked(false)
    , w_clicked(false)
    , a_clicked(false)
    , s_clicked(false)
    , d_clicked(false)
    , q_clicked(false)
    , e_clicked(false)
    , is_range(false)
    , m_speed(0.5f)
    , is_reset(false)

    , spacer_ring_radius(3.5f)
    , longitudinal_resolution(1.f)
    , transverse_resolution(10.f)
    , spacer_ply(1.f)

{
    // ���û����˫�������Ȼ���
    setFormat(QOpenGLWidget::format());
}

ContinuumShow::~ContinuumShow()
{

}

void ContinuumShow::initializeGL()
{
    // ������Ӱƽ��
    glShadeModel(GL_SMOOTH);
    // ��ɫ����
    glClearColor(0.0, 0.6, 0.6, 0.3);

    // ������Ȼ���
    glClearDepth(1.0);
    // ������Ȳ���
    glEnable(GL_DEPTH_TEST);
    // ������Ȳ��Ե�����
    glDepthFunc(GL_LEQUAL);
    // ����ϵͳ��͸�ӽ�������
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    // ����2D����ӳ��
    glEnable(GL_TEXTURE_2D);
    // ��������
    loadGLTextures();
}

void ContinuumShow::paintGL()
{
    renderScene();
    update();
}

void ContinuumShow::resizeGL(int width = 960, int height = 540)
{
    // ��ֹ���ڴ�С��Ϊ0
    if (height == 0)
    {
        height = 1;
    }
    // ���õ�ǰ���ӿ�
    glViewport(0, 0, (GLint)width, (GLint)height);
    // ѡ��ͶӰ����
    glMatrixMode(GL_PROJECTION);
    // ����ͶӰ����
    glLoadIdentity();
    // �����ӿڵĴ�С
    qgluPerspective(90.0, (GLdouble)width / (GLdouble)height, 0.99, 1000.0);
    //glOrtho(-50, 50, -50 * 9 / 16, 50 * 9 / 16, 0, 100);
    // ѡ��ģ�͹۲����
    glMatrixMode(GL_MODELVIEW);
    // ����ͶӰ����
    glLoadIdentity();
}

//���Ʊ߿�
void ContinuumShow::drawLine()
{

    glBegin(GL_LINES);
    float dx = 0.863f;
    float dy = 0.485f;
    float dz = 1.01f;
    glVertex3f(-dx, -dy, dz);
    glVertex3f(-dx, dy, dz);

    glVertex3f(-dx, dy, dz);
    glVertex3f(dx, dy, dz);
    
    glVertex3f(dx, dy, dz);
    glVertex3f(dx, -dy, dz);
    
    glVertex3f(dx, -dy, dz);
    glVertex3f(-dx, -dy, dz);
    glEnd();
}

// ����������
void ContinuumShow::drawCube()
{
    glBegin(GL_QUAD_STRIP);         //���͹�����
    glColor3f(1, 0, 0);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glColor3f(1, 1, 0);
    glVertex3f(0.0f, 1.0f, 0.0f);
    glColor3f(0, 1, 0);
    glVertex3f(1.0f, 0.0f, 0.0f);
    glColor3f(0, 1, 1);
    glVertex3f(1.0f, 1.0f, 0.0f);
    glColor3f(1, 0, 0);
    glVertex3f(1.0f, 0.0f, -1.0f);
    glColor3f(1, 1, 0);
    glVertex3f(1.0f, 1.0f, -1.0f);
    glColor3f(0, 1, 0);
    glVertex3f(0.0f, 0.0f, -1.0f);
    glColor3f(0, 1, 1);
    glVertex3f(0.0f, 1.0f, -1.0f);
    glColor3f(1, 0, 0);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glColor3f(1, 1, 0);
    glVertex3f(0.0f, 1.0f, 0.0f);
    glEnd();
    glBegin(GL_QUAD_STRIP);
    glColor3f(0, 0, 1);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glColor3f(1, 0, 1);
    glVertex3f(1.0f, 0.0f, 0.0f);
    glColor3f(0, 1, 0);
    glVertex3f(0.0f, 0.0f, -1.0f);
    glColor3f(1, 0, 0);
    glVertex3f(1.0f, 0.0f, -1.0f);
    glColor3f(1, 1, 0);
    glVertex3f(0.0f, 1.0f, 0.0f);
    glColor3f(1, 0, 1);
    glVertex3f(1.0f, 1.0f, 0.0f);
    glColor3f(0, 0, 1);
    glVertex3f(0.0f, 1.0f, -1.0f);
    glColor3f(1, 0, 0);
    glVertex3f(1.0f, 1.0f, -1.0f);
    glEnd();
}

// ����Բ��
void ContinuumShow::drawCircle(float z_f = 0.f)
{
    float tp = transverse_resolution;
    float r = spacer_ring_radius;
    glBegin(GL_TRIANGLE_FAN);           //����������������δ�
    glVertex3f(0.0f, 0.0f, z_f);
    for (size_t i = 0; i <= 360; i += (int)tp)
    {
        float p = i * 3.14 / 180;
        glVertex3f(sin(p) * r, cos(p) * r, z_f);
    }
    glEnd();
}

// ����Բ����
void ContinuumShow::drawCylinder(float z_f = 1.f)
{
    // ���������κ��ı��εȻ���ͼԪ���Ƶ���ԲԲ��������ԭ�㣬 �뾶Ϊ r����Ϊ h�������� z �᷽���Բ����
    // �����ö���ı���,�����ö������������ʾ
    float r = spacer_ring_radius;
    float tp = transverse_resolution;
    float sp = spacer_ply;
    glBegin(GL_QUAD_STRIP);//��������ı��δ�
    size_t i = 0;
    float p;
    for (i = 0; i <= 360; i += (int)tp)
    {
        p = i * PI / 180;
        glVertex3f(sin(p) * r, z_f+ sp, cos(p) * r);
        glVertex3f(sin(p) * r, z_f+0.0f, cos(p) * r);
    }
    glEnd();
    drawCircle(0.f);
    drawCircle(1.f);
}


// ��������
void ContinuumShow::loadGLTextures()
{
    QImage tex1, buf1;
    QImage tex2, buf2;
    if (!buf1.load("./data/qt-logo.jpg"))
    {
        // ������벻�ɹ����Զ�����һ��128*128��32λɫ����ɫͼƬ��
        qWarning("Could not read image file1!");
        QImage dummy(128, 128, QImage::Format_RGB32);
        dummy.fill(Qt::green);
        buf1 = dummy;
    }

    if (!buf2.load("./data/qt-wood.jpg"))
    {
        // ������벻�ɹ����Զ�����һ��128*128��32λɫ����ɫͼƬ��
        qWarning("Could not read image file2!");
        QImage dummy(128, 128, QImage::Format_RGB32);
        dummy.fill(Qt::green);
        buf2 = dummy;
    }

    //***********************************************//
    // ����0��qt-logo
    //***********************************************//
    //ת������������
    tex1 = buf1;
    // ��������
    glGenTextures(1, &texture[0]);
    // ʹ������λͼ�������ɵĵ�������,����������texture[0]�󶨵�����Ŀ����
    glBindTexture(GL_TEXTURE_2D, texture[0]);
    // WRAP�������������곬��[0,0]��[1,1]�ķ�Χ����ô�����أ�
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    // Filter��������������ӳ�䵽����λ��(127.34,255.14)����ô��?
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // ������
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    // �����������CPU����GPU��������Ϊ��ǰ����
    // �ڴ���һ����ʱ��������ã�����Ч�ʷǳ��͡�
    // ������ʱ���Բμ�����󶨡�
    glTexImage2D(GL_TEXTURE_2D, 0, 3, tex1.width(), tex1.height(), 0,
        GL_RGBA, GL_UNSIGNED_BYTE, tex1.bits());

    //***********************************************//
    // ����0��qt-logo
    //***********************************************//
    tex2 = buf2;
    glGenTextures(1, &texture[1]);
    glBindTexture(GL_TEXTURE_2D, texture[1]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, tex2.width(), tex2.height(), 0,
        GL_RGBA, GL_UNSIGNED_BYTE, tex2.bits());
    // ʹ������
    // ���ȵ���glEnable( GL_TEXTURE_2D )��������2D����;
    // Ȼ�����ͼ�Σ�����Ϊÿ������ָ��ST����;
    // ������glDisable( GL_TEXTURE_2D ).
}

// ������Ⱦ
void ContinuumShow::renderScene(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glPushMatrix();
    renderBasicShape();
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0, 0, 0);
    renderTextureCube();
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0, 0, 0);
    glColor3f(1, 0, 0);
    renderContinuumShape();
    glPopMatrix();
}

// ��Ⱦ����ͼ��
void ContinuumShow::renderBasicShape()
{

    if (is_range)
    {

    }
    else
    {
        glPushMatrix();
        glColor3f(0, 0, 1);
        glLineWidth(8.f);
        glTranslatef(0, 0, -2);
        drawLine();
        glPopMatrix();
    }
}

void ContinuumShow::renderContinuumShape()
{
    setRotateAndTranslation();
    glPushMatrix();
    glTranslatef(trans_x, trans_y, trans_z);
    glRotatef(rot_angle_x, 1.0, 0.0, 0.0);
    glRotatef(rot_angle_y, 0.0, 1.0, 0.0);
    glRotatef(0, 0.0, 0.0, 1.0);
    
    drawContinuumSpacer();
    glPopMatrix();
}



// ��Ⱦ����
void ContinuumShow::renderTextureCube()
{
    //return;
    setRotateAndTranslation();
    // ����ӳ��
    glColor3f(1.0, 1.0, 1.0);
    glTranslatef(0, 0, 0);
    glTranslatef(trans_x, trans_y, trans_z);
    glRotatef(rot_angle_x, 1.0, 0.0, 0.0);
    glRotatef(rot_angle_y, 0.0, 1.0, 0.0);
    glRotatef(0, 0.0, 0.0, 1.0);
    
    // ʹ������λͼ�������ɵĵ�������,����������texture[0]�󶨵�����Ŀ����
    glBindTexture(GL_TEXTURE_2D, texture[0]);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(-1.0, -1.0, 1.0);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(1.0, -1.0, 1.0);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(1.0, 1.0, 1.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(-1.0, 1.0, 1.0);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(-1.0, -1.0, -1.0);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(-1.0, 1.0, -1.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(1.0, 1.0, -1.0);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(1.0, -1.0, -1.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(-1.0, 1.0, -1.0);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(-1.0, 1.0, 1.0);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(1.0, 1.0, 1.0);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(1.0, 1.0, -1.0);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(-1.0, -1.0, -1.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(1.0, -1.0, -1.0);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(1.0, -1.0, 1.0);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(-1.0, -1.0, 1.0);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(1.0, -1.0, -1.0);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(1.0, 1.0, -1.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(1.0, 1.0, 1.0);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(1.0, -1.0, 1.0);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(-1.0, -1.0, -1.0);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(-1.0, -1.0, 1.0);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(-1.0, 1.0, 1.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(-1.0, 1.0, -1.0);
    glEnd();

}

void ContinuumShow::renderTextureCylinder()
{
    // ����ӳ��
    glTranslatef(0.0f, 0.0f, -12.0f);
    glRotatef(0, 1.0, 0.0, 0.0);
    //glRotatef(yRot, 0.0, 1.0, 0.0);
    //glRotatef(zRot, 0.0, 0.0, 1.0);
    // ѡ��ʹ�õ�����
    glBindTexture(GL_TEXTURE_2D, texture[1]);
    // ���������κ��ı��εȻ���ͼԪ���Ƶ���ԲԲ��������ԭ�㣬 �뾶Ϊ r����Ϊ h�������� z �᷽���Բ����
    // �����ö���ı���,�����ö������������ʾ
    glBegin(GL_QUAD_STRIP);//��������ı��δ�
    int i = 0;
    for (i = 0; i <= 360; i += 15)
    {
        float p = i * 3.14 / 180;
        //p��Բ�������Ӧ�ģ� ����������ĺ�������Բ��ɨ���ĽǶ�һ��ı䣬���ܹ�������ͼ��ˢ����ȥ�ˣ�
        //������������Ϊͼ��ĸ߶Ⱥ�����߶ȵĶ�Ӧ��������ʵĲ����Ǹ���ʵ�ʲ��Եõ���
        glTexCoord2f(p / 10, 0.1f);
        glVertex3f(sin(p), cos(p), 1.0f);   //��� 1.0fָ�����Ǹ߶�h
        glTexCoord2f(p / 10, 0.0f);
        glVertex3f(sin(p), cos(p), 0.0f);
    }
    glEnd();
    //bottom circle
    glBegin(GL_TRIANGLE_FAN);           //����������������δ�
    glTexCoord2f(0.0f, 0.0f);           //������ͼ(0, 0)ӳ�䵽Բ��
    glVertex3f(0.0f, 0.0f, 0.0f);
    for (i = 0; i <= 360; i += 15)
    {
        float p = i * 3.14 / 180;
        glTexCoord2f(1.0f, 0.0f);       //������ͼ(1, 0)ӳ�䵽Բ��
        glVertex3f(sin(p), cos(p), 0.0f);
    }
    glEnd();
    glTranslatef(0, 0, 1);              //�趨�߶�Ϊ1�����ϵ���
    //top circle
    glBegin(GL_TRIANGLE_FAN);           //����������������δ�
    glTexCoord2f(0.0f, 0.0f);           //������ͼ(0, 0)ӳ�䵽Բ��
    glVertex3f(0.0f, 0.0f, 0.0f);
    for (i = 0; i <= 360; i += 15)
    {
        float p = i * 3.14 / 180;
        glTexCoord2f(1.0f, 0.0f);       //������ͼ(1, 0)ӳ�䵽Բ��
        glVertex3f(sin(p), cos(p), 0.0f);
    }
    glEnd();
}

void ContinuumShow::setRotateAndTranslation()
{
    static int LastX = mos_x;
    static int LastY = mos_y;
    //static float tx(trans_x);
    //static float tz(trans_z);
    if (LastX != mos_x)
    {
        rot_angle_y -= 5.f*m_speed * (LastX > mos_x ? 1.f : -1.f);
        LastX = mos_x;
    }
    if (LastY != mos_y)
    {
        rot_angle_x -= 5.f*m_speed * (LastY > mos_y ? 1.f : -1.f);
        LastY = mos_y;
    }

    if (rot_angle_x > 180.f)
        rot_angle_x = rot_angle_x-360.f;
    if (rot_angle_x < -180.f)
        rot_angle_x = rot_angle_x + 360.f;
    if (rot_angle_y > 180.f)
        rot_angle_y = rot_angle_y - 360.f;
    if (rot_angle_y < -180.f)
        rot_angle_y = rot_angle_y + 360.f;
    if(is_range)
    {
        if (w_clicked)
            trans_z -= m_speed;
        if (s_clicked)
            trans_z += m_speed;
        if (a_clicked)
            trans_x -= m_speed;
        if (d_clicked)
            trans_x += m_speed;
        if (q_clicked)
            trans_y += m_speed;
        if (e_clicked)
            trans_y -= m_speed;
    }

    if (is_reset)
    {
        trans_x = 0.f;
        trans_y = 0.f;
        trans_z = -10.f;
        rot_angle_x = 0.f;
        rot_angle_y = 0.f;
    }

    w_clicked = false; a_clicked = false; s_clicked = false;
    d_clicked = false; q_clicked = false; e_clicked = false;
}


void ContinuumShow::drawContinuumSpacer()
{

}

void ContinuumShow::forwardKinematics()
{

}
