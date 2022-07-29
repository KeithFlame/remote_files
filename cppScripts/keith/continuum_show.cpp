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
    // 设置画面的双缓冲和深度缓存
    setFormat(QOpenGLWidget::format());
}

ContinuumShow::~ContinuumShow()
{

}

void ContinuumShow::initializeGL()
{
    // 启用阴影平滑
    glShadeModel(GL_SMOOTH);
    // 白色背景
    glClearColor(0.0, 0.6, 0.6, 0.3);

    // 设置深度缓存
    glClearDepth(1.0);
    // 启用深度测试
    glEnable(GL_DEPTH_TEST);
    // 所作深度测试的类型
    glDepthFunc(GL_LEQUAL);
    // 告诉系统对透视进行修正
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    // 启用2D纹理映射
    glEnable(GL_TEXTURE_2D);
    // 加载纹理
    loadGLTextures();
}

void ContinuumShow::paintGL()
{
    renderScene();
    update();
}

void ContinuumShow::resizeGL(int width = 960, int height = 540)
{
    // 防止窗口大小变为0
    if (height == 0)
    {
        height = 1;
    }
    // 重置当前的视口
    glViewport(0, 0, (GLint)width, (GLint)height);
    // 选择投影矩阵
    glMatrixMode(GL_PROJECTION);
    // 重置投影矩阵
    glLoadIdentity();
    // 设置视口的大小
    qgluPerspective(90.0, (GLdouble)width / (GLdouble)height, 0.99, 1000.0);
    //glOrtho(-50, 50, -50 * 9 / 16, 50 * 9 / 16, 0, 100);
    // 选择模型观察矩阵
    glMatrixMode(GL_MODELVIEW);
    // 重置投影矩阵
    glLoadIdentity();
}

//绘制边框
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

// 绘制立方体
void ContinuumShow::drawCube()
{
    glBegin(GL_QUAD_STRIP);         //填充凸多边形
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

// 绘制圆形
void ContinuumShow::drawCircle(float z_f = 0.f)
{
    float tp = transverse_resolution;
    float r = spacer_ring_radius;
    glBegin(GL_TRIANGLE_FAN);           //扇形连续填充三角形串
    glVertex3f(0.0f, 0.0f, z_f);
    for (size_t i = 0; i <= 360; i += (int)tp)
    {
        float p = i * 3.14 / 180;
        glVertex3f(sin(p) * r, cos(p) * r, z_f);
    }
    glEnd();
}

// 绘制圆柱体
void ContinuumShow::drawCylinder(float z_f = 1.f)
{
    // 利用三角形和四边形等基本图元绘制底面圆圆心在坐标原点， 半径为 r，高为 h，方向沿 z 轴方向的圆柱；
    // 侧面用多个四边形,底面用多个三角形来表示
    float r = spacer_ring_radius;
    float tp = transverse_resolution;
    float sp = spacer_ply;
    glBegin(GL_QUAD_STRIP);//连续填充四边形串
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


// 加载纹理
void ContinuumShow::loadGLTextures()
{
    QImage tex1, buf1;
    QImage tex2, buf2;
    if (!buf1.load("./data/qt-logo.jpg"))
    {
        // 如果载入不成功，自动生成一个128*128的32位色的绿色图片。
        qWarning("Could not read image file1!");
        QImage dummy(128, 128, QImage::Format_RGB32);
        dummy.fill(Qt::green);
        buf1 = dummy;
    }

    if (!buf2.load("./data/qt-wood.jpg"))
    {
        // 如果载入不成功，自动生成一个128*128的32位色的绿色图片。
        qWarning("Could not read image file2!");
        QImage dummy(128, 128, QImage::Format_RGB32);
        dummy.fill(Qt::green);
        buf2 = dummy;
    }

    //***********************************************//
    // 纹理0：qt-logo
    //***********************************************//
    //转换成纹理类型
    tex1 = buf1;
    // 创建纹理
    glGenTextures(1, &texture[0]);
    // 使用来自位图数据生成的典型纹理,将纹理名字texture[0]绑定到纹理目标上
    glBindTexture(GL_TEXTURE_2D, texture[0]);
    // WRAP参数：纹理坐标超出[0,0]到[1,1]的范围该怎么处理呢？
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    // Filter参数：纹理坐标映射到纹素位置(127.34,255.14)该怎么办?
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // 纹理环境
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    // 将纹素数组从CPU传至GPU并且设置为当前纹理。
    // 在处理单一纹理时，你可以用，负责效率非常低。
    // 多纹理时可以参见纹理绑定。
    glTexImage2D(GL_TEXTURE_2D, 0, 3, tex1.width(), tex1.height(), 0,
        GL_RGBA, GL_UNSIGNED_BYTE, tex1.bits());

    //***********************************************//
    // 纹理0：qt-logo
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
    // 使用纹理
    // 首先调用glEnable( GL_TEXTURE_2D )，来启用2D纹理;
    // 然后绘制图形，并且为每个顶点指定ST坐标;
    // 最后调用glDisable( GL_TEXTURE_2D ).
}

// 场景渲染
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

// 渲染基本图形
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



// 渲染纹理
void ContinuumShow::renderTextureCube()
{
    //return;
    setRotateAndTranslation();
    // 纹理映射
    glColor3f(1.0, 1.0, 1.0);
    glTranslatef(0, 0, 0);
    glTranslatef(trans_x, trans_y, trans_z);
    glRotatef(rot_angle_x, 1.0, 0.0, 0.0);
    glRotatef(rot_angle_y, 0.0, 1.0, 0.0);
    glRotatef(0, 0.0, 0.0, 1.0);
    
    // 使用来自位图数据生成的典型纹理,将纹理名字texture[0]绑定到纹理目标上
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
    // 纹理映射
    glTranslatef(0.0f, 0.0f, -12.0f);
    glRotatef(0, 1.0, 0.0, 0.0);
    //glRotatef(yRot, 0.0, 1.0, 0.0);
    //glRotatef(zRot, 0.0, 0.0, 1.0);
    // 选择使用的纹理
    glBindTexture(GL_TEXTURE_2D, texture[1]);
    // 利用三角形和四边形等基本图元绘制底面圆圆心在坐标原点， 半径为 r，高为 h，方向沿 z 轴方向的圆柱；
    // 侧面用多个四边形,底面用多个三角形来表示
    glBegin(GL_QUAD_STRIP);//连续填充四边形串
    int i = 0;
    for (i = 0; i <= 360; i += 15)
    {
        float p = i * 3.14 / 180;
        //p和圆周是相对应的， 这里让纹理的横坐标随圆周扫过的角度一起改变，就能够将纹理图“刷”上去了，
        //而纵坐标设置为图像的高度和纹理高度的对应，这里合适的参数是根据实际测试得到的
        glTexCoord2f(p / 10, 0.1f);
        glVertex3f(sin(p), cos(p), 1.0f);   //这个 1.0f指定的是高度h
        glTexCoord2f(p / 10, 0.0f);
        glVertex3f(sin(p), cos(p), 0.0f);
    }
    glEnd();
    //bottom circle
    glBegin(GL_TRIANGLE_FAN);           //扇形连续填充三角形串
    glTexCoord2f(0.0f, 0.0f);           //将纹理图(0, 0)映射到圆心
    glVertex3f(0.0f, 0.0f, 0.0f);
    for (i = 0; i <= 360; i += 15)
    {
        float p = i * 3.14 / 180;
        glTexCoord2f(1.0f, 0.0f);       //将纹理图(1, 0)映射到圆周
        glVertex3f(sin(p), cos(p), 0.0f);
    }
    glEnd();
    glTranslatef(0, 0, 1);              //设定高度为1，画上底面
    //top circle
    glBegin(GL_TRIANGLE_FAN);           //扇形连续填充三角形串
    glTexCoord2f(0.0f, 0.0f);           //将纹理图(0, 0)映射到圆心
    glVertex3f(0.0f, 0.0f, 0.0f);
    for (i = 0; i <= 360; i += 15)
    {
        float p = i * 3.14 / 180;
        glTexCoord2f(1.0f, 0.0f);       //将纹理图(1, 0)映射到圆周
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
