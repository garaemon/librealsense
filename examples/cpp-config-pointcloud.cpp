#include <librealsense/rs.hpp>
#include "example.hpp"

#include <cstdarg>
#include <thread>
#include <iostream>
#include <algorithm>
#pragma comment(lib, "opengl32.lib")

inline void glVertex(const rs::float3 & vertex) { glVertex3fv(&vertex.x); }
inline void glTexCoord(const rs::float2 & tex_coord) { glTexCoord2fv(&tex_coord.x); }

struct state { double yaw, pitch, lastX, lastY; bool ml; std::vector<rs::stream> tex_streams; int index; rs::device * dev; };


struct int2 { int x,y; };
struct rect
{
    int x0,y0,x1,y1;
    bool contains(const int2 & p) const { return x0 <= p.x && y0 <= p.y && p.x < x1 && p.y < y1; }
    rect shrink(int amt) const { return {x0+amt, y0+amt, x1-amt, y1-amt}; }
};
struct color { float r,g,b; };

struct gui
{
    int2 cursor, clicked_offset, scroll_vec;
    bool click, mouse_down;
    int clicked_id;

    gui() : scroll_vec({0,0}), click(), mouse_down(), clicked_id() {}

    void label(const int2 & p, const color & c, const char * format, ...)
    {
        va_list args;
        va_start(args, format);
        char buffer[1024];
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        glColor3f(c.r, c.g, c.b);
        draw_text(p.x, p.y, buffer);
    }

    void fill_rect(const rect & r, const color & c)
    {
        glBegin(GL_QUADS);
        glColor3f(c.r, c.g, c.b);
        glVertex2i(r.x0, r.y0);
        glVertex2i(r.x0, r.y1);
        glVertex2i(r.x1, r.y1);
        glVertex2i(r.x1, r.y0);
        glEnd();
    }

    bool button(const rect & r, const std::string & label)
    {
        fill_rect(r, {1,1,1});
        fill_rect(r.shrink(2), r.contains(cursor) ? (mouse_down ? color{0.3f,0.3f,0.3f} : color{0.4f,0.4f,0.4f}) : color{0.5f,0.5f,0.5f});
        glColor3f(1,1,1);
        draw_text(r.x0 + 4, r.y1 - 8, label.c_str());
        return click && r.contains(cursor);
    }

    bool checkbox(const rect & r, bool & value)
    {
        bool changed = false;
        if(click && r.contains(cursor))
        {
            value = !value;
            changed = true;
        }
        fill_rect(r, {1,1,1});
        fill_rect(r.shrink(1), {0.5,0.5,0.5});
        if(value) fill_rect(r.shrink(3), {1,1,1});
        return changed;
    }

    bool slider(int id, const rect & r, double min, double max, double step, double & value)
    {
        bool changed = false;
        const int w = r.x1 - r.x0, h = r.y1 - r.y0;
        double p = (w - h) * (value - min) / (max - min);
        if(mouse_down && clicked_id == id)
        {
            p = std::max(0.0, std::min<double>(cursor.x - clicked_offset.x - r.x0, w-h));
            double new_value = min + p * (max - min) / (w - h);
            if(step) new_value = std::round((new_value - min) / step) * step + min;
            changed = new_value != value;
            value = new_value;
            p = (w - h) * (value - min) / (max - min);
        }
        const rect dragger = {int(r.x0+p), int(r.y0), int(r.x0+p+h), int(r.y1)};
        if(click && dragger.contains(cursor))
        {
            clicked_offset = {cursor.x - dragger.x0, cursor.y - dragger.y0};
            clicked_id = id;
        }
        fill_rect(r, {0.5,0.5,0.5});
        fill_rect(dragger, {1,1,1});
        return changed;
    }

    void vscroll(const rect & r, int client_height, int & offset)
    {
        if(r.contains(cursor)) offset -= scroll_vec.y * 20;
        offset = std::min(offset, client_height - (r.y1 - r.y0));
        offset = std::max(offset, 0);
        if(client_height <= r.y1 - r.y0) return;
        auto bar = r; bar.x0 = bar.x1 - 10;
        auto dragger = bar;
        dragger.y0 = bar.y0 + offset * (r.y1 - r.y0) / client_height;
        dragger.y1 = bar.y0 + (offset + r.y1 - r.y0) * (r.y1 - r.y0) / client_height;
        fill_rect(bar, {0.5,0.5,0.5});
        fill_rect(dragger, {1,1,1});
    }
};

texture_buffer buffers[4];

int main(int argc, char * argv[]) try
{
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    glfwInit();
    auto win_2d = glfwCreateWindow(1550, 960, "CPP Configuration Example", nullptr, nullptr);
    glfwMakeContextCurrent(win_2d);
    gui g;
    glfwSetWindowUserPointer(win_2d, &g);
    glfwSetCursorPosCallback(win_2d, [](GLFWwindow * w, double cx, double cy) { reinterpret_cast<gui *>(glfwGetWindowUserPointer(w))->cursor = {(int)cx, (int)cy}; });
    glfwSetScrollCallback(win_2d, [](GLFWwindow * w, double x, double y) { reinterpret_cast<gui *>(glfwGetWindowUserPointer(w))->scroll_vec = {(int)x, (int)y}; });
    glfwSetMouseButtonCallback(win_2d, [](GLFWwindow * w, int button, int action, int mods)
    {
        auto g = reinterpret_cast<gui *>(glfwGetWindowUserPointer(w));
        if(action == GLFW_PRESS)
        {
            g->clicked_id = 0;
            g->click = true;
        }
        g->mouse_down = action != GLFW_RELEASE;
    });

    rs::context ctx;
    if(ctx.get_device_count() < 1) throw std::runtime_error("No device found. Is it plugged in?");

    rs::device * dev = ctx.get_device(0);
    dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev->enable_stream(rs::stream::color, rs::preset::best_quality);
    dev->enable_stream(rs::stream::infrared, rs::preset::best_quality);
    try { dev->enable_stream(rs::stream::infrared2, rs::preset::best_quality); } catch(...) {}

    state app_state = {0, 0, 0, 0, false, {rs::stream::color, rs::stream::depth, rs::stream::infrared}, 0, dev};
    if(dev->is_stream_enabled(rs::stream::infrared2)) app_state.tex_streams.push_back(rs::stream::infrared2);

    //std::this_thread::sleep_for(std::chrono::seconds(1));

    struct option { rs::option opt; double min, max, step, value; };
    std::vector<option> options;
    for(int i=0; i<RS_OPTION_COUNT; ++i)
    {
        option o = {(rs::option)i};
        if(!dev->supports_option(o.opt)) continue;
        dev->get_option_range(o.opt, o.min, o.max, o.step);
        if(o.min == o.max) continue;
        try { o.value = dev->get_option(o.opt); } catch(...) {}
        options.push_back(o);
    }
    double dc_preset = 0, iv_preset = 0;

    GLFWwindow * win = glfwCreateWindow(640, 480, "pointcloud", 0, 0);
    glfwSetWindowUserPointer(win, &app_state);

    glfwSetMouseButtonCallback(win, [](GLFWwindow * win, int button, int action, int mods)
    {
        auto s = (state *)glfwGetWindowUserPointer(win);
        if(button == GLFW_MOUSE_BUTTON_LEFT) s->ml = action == GLFW_PRESS;
        if(button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) s->index = (s->index+1) % s->tex_streams.size();
    });

    glfwSetCursorPosCallback(win, [](GLFWwindow * win, double x, double y)
    {
        auto s = (state *)glfwGetWindowUserPointer(win);
        if(s->ml)
        {
            s->yaw -= (x - s->lastX);
            s->yaw = std::max(s->yaw, -120.0);
            s->yaw = std::min(s->yaw, +120.0);
            s->pitch += (y - s->lastY);
            s->pitch = std::max(s->pitch, -80.0);
            s->pitch = std::min(s->pitch, +80.0);
        }
        s->lastX = x;
        s->lastY = y;
    });

    glfwSetKeyCallback(win, [](GLFWwindow * win, int key, int scancode, int action, int mods)
    {
        auto s = (state *)glfwGetWindowUserPointer(win);
        if (action == GLFW_RELEASE)
        {
            if (key == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(win, 1);
            else if (key == GLFW_KEY_F1)
            {
               if (!s->dev->is_streaming()) s->dev->start();
            }
            else if (key == GLFW_KEY_F2)
            {
               if (s->dev->is_streaming()) s->dev->stop();
            }
        }
    });

    glfwMakeContextCurrent(win);
    texture_buffer tex;

    int frames = 0; float time = 0, fps = 0;
    auto t0 = std::chrono::high_resolution_clock::now();

    int offset = 0, panel_height = 1;
    while(!glfwWindowShouldClose(win_2d) &&
          !glfwWindowShouldClose(win))
    {
        glfwPollEvents();
        if(dev->is_streaming()) dev->wait_for_frames();

        int w, h;
        glfwGetFramebufferSize(win_2d, &w, &h);
        glViewport(0, 0, w, h);
        glClear(GL_COLOR_BUFFER_BIT);

        glfwGetWindowSize(win_2d, &w, &h);
        glLoadIdentity();
        glOrtho(0, w, h, 0, -1, +1);

        g.vscroll({w-270, 0, w, h}, panel_height, offset);
        int y = 10 - offset;

        if(dev->is_streaming())
        {
            if(g.button({w-260, y, w-20, y+24}, "Stop Capture")) dev->stop();
        }
        else
        {
            if(g.button({w-260, y, w-20, y+24}, "Start Capture")) dev->start();
        }
        y += 34;
        if(!dev->is_streaming())
        {
            for(int i=0; i<4; ++i)
            {
                auto s = (rs::stream)i;
                bool enable = dev->is_stream_enabled(s);
                if(g.checkbox({w-260, y, w-240, y+20}, enable))
                {
                    if(enable) dev->enable_stream(s, rs::preset::best_quality);
                    else dev->disable_stream(s);
                }
                g.label({w-234, y+13}, {1,1,1}, "Enable %s", rs_stream_to_string((rs_stream)i));
                y += 30;
            }
        }

        for(auto & o : options)
        {
            std::ostringstream ss; ss << o.opt << ": " << o.value;
            g.label({w-260,y+12}, {1,1,1}, ss.str().c_str());
            if(g.slider((int)o.opt + 1, {w-260,y+16,w-20,y+36}, o.min, o.max, o.step, o.value))
            {
                dev->set_option(o.opt, o.value);
            }
            y += 38;
        }
        g.label({w-260,y+12}, {1,1,1}, "Depth control parameters preset: %g", dc_preset);
        if(g.slider(100, {w-260,y+16,w-20,y+36}, 0, 5, 1, dc_preset)) apply_depth_control_preset(dev, static_cast<int>(dc_preset));
        y += 38;
        g.label({w-260,y+12}, {1,1,1}, "IVCAM options preset: %g", iv_preset);
        if(g.slider(101, {w-260,y+16,w-20,y+36}, 0, 8, 1, iv_preset)) apply_ivcam_preset(dev, static_cast<int>(iv_preset));
        y += 38;

        panel_height = y + 10 + offset;

        if(dev->is_streaming())
        {
            w-=270;
            buffers[0].show(*dev, rs::stream::color, 0, 0, w/2, h/2);
            buffers[1].show(*dev, rs::stream::depth, w/2, 0, w-w/2, h/2);
            buffers[2].show(*dev, rs::stream::infrared, 0, h/2, w/2, h-h/2);
            buffers[3].show(*dev, rs::stream::infrared2, w/2, h/2, w-w/2, h-h/2);
        }

        glfwSwapBuffers(win_2d);

        g.scroll_vec = {0,0};
        g.click = false;
        if(!g.mouse_down) g.clicked_id = 0;


        // pointcloud
                auto t1 = std::chrono::high_resolution_clock::now();
        time += std::chrono::duration<float>(t1-t0).count();
        t0 = t1;
        ++frames;
        if(time > 0.5f)
        {
            fps = frames / time;
            frames = 0;
            time = 0;
        }

        const rs::stream tex_stream = app_state.tex_streams[app_state.index];
        const float depth_scale = dev->get_depth_scale();
        const rs::extrinsics extrin = dev->get_extrinsics(rs::stream::depth, tex_stream);
        const rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
        const rs::intrinsics tex_intrin = dev->get_stream_intrinsics(tex_stream);
        bool identical = depth_intrin == tex_intrin && extrin.is_identity();

        glPushAttrib(GL_ALL_ATTRIB_BITS);

        tex.upload(*dev, tex_stream);

        int width, height;
        glfwGetFramebufferSize(win, &width, &height);
        glViewport(0, 0, width, height);
        glClearColor(52.0f/255, 72.f/255, 94.0f/255.0f, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        gluPerspective(60, (float)width/height, 0.01f, 20.0f);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        gluLookAt(0,0,0, 0,0,1, 0,-1,0);

        glTranslatef(0,0,+0.5f);
        glRotated(app_state.pitch, 1, 0, 0);
        glRotated(app_state.yaw, 0, 1, 0);
        glTranslatef(0,0,-0.5f);

        glPointSize((float)width/640);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, tex.get_gl_handle());
        glBegin(GL_POINTS);

        auto points = reinterpret_cast<const rs::float3 *>(dev->get_frame_data(rs::stream::points));
        auto depth = reinterpret_cast<const uint16_t *>(dev->get_frame_data(rs::stream::depth));

        for(int y=0; y<depth_intrin.height; ++y)
        {
            for(int x=0; x<depth_intrin.width; ++x)
            {
                if(points->z) //if(uint16_t d = *depth++)
                {
                    //const rs::float3 point = depth_intrin.deproject({static_cast<float>(x),static_cast<float>(y)}, d*depth_scale);
                    glTexCoord(identical ? tex_intrin.pixel_to_texcoord({static_cast<float>(x),static_cast<float>(y)}) : tex_intrin.project_to_texcoord(extrin.transform(*points)));
                    glVertex(*points);
                }
                ++points;
            }
        }
        glEnd();
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glPopAttrib();

        glfwGetWindowSize(win, &width, &height);
        glPushAttrib(GL_ALL_ATTRIB_BITS);
        glPushMatrix();
        glOrtho(0, width, height, 0, -1, +1);

        std::ostringstream ss; ss << dev->get_name() << " (" << app_state.tex_streams[app_state.index] << ")";
        draw_text((width-get_text_width(ss.str().c_str()))/2, height-20, ss.str().c_str());

        ss.str(""); ss << fps << " FPS";
        draw_text(20, 40, ss.str().c_str());
        glPopMatrix();

        glfwSwapBuffers(win);

        int z = 99;


    }

    glfwTerminate();
    return 0;
}
catch(const rs::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch(const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
