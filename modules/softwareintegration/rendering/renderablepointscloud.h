/*****************************************************************************************
 *                                                                                       *
 * OpenSpace                                                                             *
 *                                                                                       *
 * Copyright (c) 2014-2020                                                               *
 *                                                                                       *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this  *
 * software and associated documentation files (the "Software"), to deal in the Software *
 * without restriction, including without limitation the rights to use, copy, modify,    *
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to    *
 * permit persons to whom the Software is furnished to do so, subject to the following   *
 * conditions:                                                                           *
 *                                                                                       *
 * The above copyright notice and this permission notice shall be included in all copies *
 * or substantial portions of the Software.                                              *
 *                                                                                       *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,   *
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A         *
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT    *
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF  *
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE  *
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                                         *
 ****************************************************************************************/

#ifndef __OPENSPACE_MODULE_SOFTWAREINTEGRATION___RENDERABLEPOINTSCLOUD___H__
#define __OPENSPACE_MODULE_SOFTWAREINTEGRATION___RENDERABLEPOINTSCLOUD___H__

#include <openspace/rendering/renderable.h>

#include <openspace/properties/scalar/floatproperty.h>
#include <openspace/properties/vector/vec3property.h>
#include <ghoul/opengl/ghoul_gl.h>

namespace ghoul::filesystem { class File; }
namespace ghoul::opengl {
    class ProgramObject;
} // namespace ghoul::opengl

namespace openspace::documentation { struct Documentation; }

namespace openspace {

    class RenderablePointsCloud : public Renderable {
    public:
        RenderablePointsCloud(const ghoul::Dictionary& dictionary);

        void initialize() override;
        void initializeGL() override;
        void deinitializeGL() override;

        bool isReady() const override;

        void render(const RenderData& data, RendererTasks& rendererTask) override;
        void update(const UpdateData& data) override;

        static documentation::Documentation Documentation();

    protected:

        void createDataSlice();
        bool loadData();
        bool loadSpeckData();
        bool readSpeckFile();
        bool loadCachedFile(const std::string& file);
        bool saveCachedFile(const std::string& file) const;

        bool _hasSpeckFile = false;
        bool _isDirty = true;

        std::unique_ptr<ghoul::opengl::ProgramObject> _shaderProgram = nullptr;

        properties::BoolProperty _toggleVisibility;
        properties::FloatProperty _opacity;
        properties::FloatProperty _size;
        properties::Vec3Property _color;

        std::string _speckFile;
        std::vector<float> _fullData;
        std::vector<float> _slicedData;

        int _nValuesPerPoints = 0;

        glm::dmat4 _transformationMatrix = glm::dmat4(1.0);

        GLuint _vertexArrayObjectID = 0;
        GLuint _vertexBufferObjectID = 0;
    };

}// namespace openspace

#endif // __OPENSPACE_MODULE_SOFTWAREINTEGRATION___RENDERABLEPOINTSCLOUD___H__
