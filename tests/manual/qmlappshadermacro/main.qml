/****************************************************************************
**
** Copyright (C) 2020 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the QtShaderTools module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

import QtQuick 2.15

Item {
    Text {
        color: "#ffffff"
        style: Text.Outline
        styleColor: "#606060"
        font.pixelSize: 28
        property int api: GraphicsInfo.api
        text: {
            if (GraphicsInfo.api === GraphicsInfo.OpenGLRhi)
                "OpenGL on QRhi";
            else if (GraphicsInfo.api === GraphicsInfo.Direct3D11Rhi)
                "D3D11 on QRhi";
            else if (GraphicsInfo.api === GraphicsInfo.VulkanRhi)
                "Vulkan on QRhi";
            else if (GraphicsInfo.api === GraphicsInfo.MetalRhi)
                "Metal on QRhi";
            else if (GraphicsInfo.api === GraphicsInfo.Null)
                "Null on QRhi";
            else
                "Unknown API";
        }
    }

    Text {
        id: srcItem
        text: "Hello world"
        anchors.centerIn: parent
        color: "red"
        font.pixelSize: 40
    }

    ShaderEffectSource {
        id: src
        sourceItem: srcItem
        anchors.fill: srcItem
        hideSource: true
        visible: false
    }

    ShaderEffect {
        anchors.fill: src
        property variant source: src
        property real amplitude: 0.04
        property real frequency: 5
        property real time: 0
        NumberAnimation on time { from: 0; to: Math.PI * 2; duration: 600; loops: -1 }
        fragmentShader: "wobble.frag.qsb"
    }
}
