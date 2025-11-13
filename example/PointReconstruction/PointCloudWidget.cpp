#include "PointCloudWidget.h"

//=======================================================
// --- Projection ---
// projection_.setToIdentity();
// projection_(0,0) = 2.0f * fx_ / imgWidth_;
// projection_(1,1) = 2.0f * fy_ / imgHeight_;
// projection_(0,2) = 1.0f - 2.0f * cx_ / imgWidth_;
// projection_(1,2) = 2.0f * cy_ / imgHeight_ - 1.0f;
// projection_(2,2) = -(pfar + pnear) / (pfar - pnear);
// projection_(2,3) = -2 * pfar * pnear / (pfar - pnear);
// projection_(3,2) = -1.0f;
// projection_(3,3) = 0.0f;
// view_.setToIdentity();
// view_.lookAt(QVector3D(0,0,0), QVector3D(0,0,1), QVector3D(0,1,0));

// QMatrix4x4 flip;
// flip.scale(-1.0f, -1.0f, 1.0f);
// view_ = flip * view_;
// =======================================================

PointCloudWidget::PointCloudWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{

    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);


    model_.setToIdentity();

}

void PointCloudWidget::setCameraIntrinsics(const Fringe::SystemParams& params, int imgWidth, int imgHeight)
{
    fx_ = static_cast<float>(params.cameraparam[0][0]);
    fy_ = static_cast<float>(params.cameraparam[1][1]);
    cx_ = static_cast<float>(params.cameraparam[0][2]);
    cy_ = static_cast<float>(params.cameraparam[1][2]);
    imgWidth_ = imgWidth;
    imgHeight_ = imgHeight;

    updateProjection();
}

void PointCloudWidget::initializeGL()
{
    initializeOpenGLFunctions();


    glEnable(GL_DEPTH_TEST);       // start depth
    glEnable(GL_PROGRAM_POINT_SIZE); // allow set point size
    glPointSize(pointSize_);       // set default point size

    // setup shaders
    setupShaders();

    vao_.create();  // glGenVertexArrays(1, &vaoID);
    vao_.bind();    // glBindVertexArray(vaoID)


    vbo_.create();  //glGenBuffers() 
    vbo_.setUsagePattern(QOpenGLBuffer::DynamicDraw);  //more update than StaticDraw  "staticDraw DynamicDraw StramDraw"

    vao_.release();
    vbo_.release();
}


void PointCloudWidget::setPointCloud(const Fringe::PointCloud& cloud)
{
    cpuBuffer_.clear();
    cpuBuffer_.reserve(cloud.points.size() * 3); 

    for (const auto& p : cloud.points)
    {
        cpuBuffer_.push_back(static_cast<float>(p.x));
        cpuBuffer_.push_back(static_cast<float>(p.y));
        cpuBuffer_.push_back(static_cast<float>(p.z));
    }

    pointCount_ = cloud.points.size();
    bufferDirty_ = true;

    center_ = computePointCloudCenterFromBuffer();
    
    computePointCloudZRangeFromBuffer(zMin_, zMax_);

    float zRange = zMax_ - zMin_;
    float padding = 0.1f * zRange;
    distance_ = 1.5f * zRange;      
    pnear = 0.1f;                       
    pfar  = distance_ + zRange + padding; 

    updateProjection(); 
    update(); 
}

void PointCloudWidget::uploadBuffer()
{
    if (!bufferDirty_ || cpuBuffer_.empty())
        return;

    vao_.bind();   

    vbo_.bind(); 

    //glBufferData(GL_ARRAY_BUFFER, size, data, usage);
    //vbo_.allocate(cpuBuffer_.data(), cpuBuffer_.size() * sizeof(float));
    if (cpuBuffer_.size() > vbo_.size()) {
    vbo_.allocate(cpuBuffer_.data(), cpuBuffer_.size() * sizeof(float));
    } else {
        vbo_.write(0, cpuBuffer_.data(), cpuBuffer_.size() * sizeof(float));
    }

    program_->bind();  //glUseProgram(programID); use shader 
    program_->enableAttributeArray(0);  //location 0 

    //glVertexAttribPointer(index, size, type, normalized, stride, pointer);
    //(location, type, offset, point number, buff size)
    program_->setAttributeBuffer(0, GL_FLOAT, 0, 3, 3 * sizeof(float));   

    program_->release();

    vbo_.release();
    vao_.release();

    bufferDirty_ = false;
}

void PointCloudWidget::setupShaders()
{
    program_ = new QOpenGLShaderProgram(this);

    // Vertex Shader
    const char* vertexShaderSrc = R"(
        #version 330 core
        layout(location = 0) in vec3 position;

        uniform mat4 model;
        uniform mat4 view;
        uniform mat4 projection;

        out float depthColor;

        void main() {
            gl_Position = projection * view * model * vec4(position, 1.0);
            depthColor = position.z;
        }
    )";

    // Fragment Shader
    const char* fragmentShaderSrc = R"(
        #version 330 core
        in float depthColor;
        out vec4 FragColor;

        uniform float zMin;
        uniform float zMax;

        void main() {
            float color = 1.0 - (depthColor - zMin) / (zMax - zMin);
            color = clamp(color, 0.0, 1.0);
            FragColor = vec4(color, color, color, 1.0);
        }
    )";
    program_->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSrc);
    program_->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSrc);
    program_->link();
}

void PointCloudWidget::paintGL()
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);  //background color 
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); //clear buff clear depth

    uploadBuffer(); // update buffer

    if (pointCount_ == 0)  
        return;


    program_->bind();  //activate shader 

    // update martrix to gpu
    program_->setUniformValue("model", model_);  
    program_->setUniformValue("view", view_);
    program_->setUniformValue("projection", projection_);

    program_->setUniformValue("zMin", zMin_-0.0f*(zMax_-zMin_));
    program_->setUniformValue("zMax", zMax_+0.0f*(zMax_-zMin_));

    vao_.bind();
    glDrawArrays(GL_POINTS, 0, pointCount_);  //paint point 
    vao_.release();

    program_->release();

}

void PointCloudWidget::resizeGL(int w, int h) //qt will use when widget changed
{
    Q_UNUSED(w)
    Q_UNUSED(h)
    updateProjection();
}

void PointCloudWidget::updateProjection()
{
    int w = width();
    int h = height();
    if (w <= 0 || h <= 0) return;

    float aspect = float(w) / float(h); 
    projection_.setToIdentity();
    projection_.perspective(35.0f, aspect, pnear, pfar);

    

    view_.setToIdentity();

    QMatrix4x4 rot;
    rot.rotate(pitch_ * 180.0f / M_PI, 1, 0, 0); 
    rot.rotate(yaw_   * 180.0f / M_PI, 0, 1, 0); 

    QVector3D eye = center_ + rot * QVector3D(0, 0, distance_);
    QVector3D up  = rot * QVector3D(0, 1, 0);

    eye += QVector3D(panX_, panY_, 0);
    QVector3D target = center_ + QVector3D(panX_, panY_, 0);

    QMatrix4x4 flip;
    flip.scale(1.0f, -1.0f, 1.0f);
    view_ = flip * view_;
    view_.lookAt(eye, target, up);


    qDebug() << "pnear:" << pnear << "pfar:" << pfar << "distance:" << distance_;
}

QVector3D PointCloudWidget::computePointCloudCenterFromBuffer()
{
    if (cpuBuffer_.empty()) 
        return QVector3D(0,0,0);

    double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
    int count = static_cast<int>(cpuBuffer_.size()/3);

    for (int i = 0; i < count; ++i) {
        sumX += cpuBuffer_[i*3 + 0];
        sumY += cpuBuffer_[i*3 + 1];
        sumZ += cpuBuffer_[i*3 + 2];
    }

    return QVector3D(sumX/count, sumY/count, sumZ/count);
}

void PointCloudWidget::computePointCloudZRangeFromBuffer(float &zMin, float &zMax)
{
    if (cpuBuffer_.empty()) {
        zMin = 0.0f; 
        zMax = 1.0f;
        return;
    }

    zMin = zMax = cpuBuffer_[2]; 
    int count = static_cast<int>(cpuBuffer_.size()/3);

    for (int i = 0; i < count; ++i) {
        float z = cpuBuffer_[i*3 + 2];
        if (z < zMin) zMin = z;
        if (z > zMax) zMax = z;
    }
}

void PointCloudWidget::mousePressEvent(QMouseEvent* event)
{
    lastMousePos_ = event->pos();
}

void PointCloudWidget::mouseMoveEvent(QMouseEvent* event)
{
    QPoint delta = event->pos() - lastMousePos_;
    lastMousePos_ = event->pos();

    Qt::MouseButtons buttons = event->buttons(); 

    if (buttons & Qt::LeftButton) {
        panX_ += delta.x() * 0.01f;
        panY_ -= delta.y() * 0.01f;
    }

    if (buttons & Qt::RightButton) {
        yaw_   += delta.x() * 0.5f * M_PI / 180.0f;
        pitch_ += delta.y() * 0.5f * M_PI / 180.0f;

        if (pitch_ > M_PI/2) pitch_ = M_PI/2;
        if (pitch_ < -M_PI/2) pitch_ = -M_PI/2;
    }

    updateProjection();
    update();
}

void PointCloudWidget::wheelEvent(QWheelEvent* event)
{
    float delta = event->angleDelta().y() / 120.0f; 
    distance_ *= (delta > 0) ? 0.9f : 1.1f;        
    if (distance_ < 0.1f) distance_ = 0.1f;
    updateProjection();
    update();
}