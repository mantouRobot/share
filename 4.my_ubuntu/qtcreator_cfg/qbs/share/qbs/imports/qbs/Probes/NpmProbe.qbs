/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing
**
** This file is part of the Qt Build Suite.
**
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms and
** conditions see http://www.qt.io/terms-conditions. For further information
** use the contact form at http://www.qt.io/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 or version 3 as published by the Free
** Software Foundation and appearing in the file LICENSE.LGPLv21 and
** LICENSE.LGPLv3 included in the packaging of this file.  Please review the
** following information to ensure the GNU Lesser General Public License
** requirements will be met: https://www.gnu.org/licenses/lgpl.html and
** http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** In addition, as a special exception, The Qt Company gives you certain additional
** rights.  These rights are described in The Qt Company LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
****************************************************************************/

import qbs
import "path-probe.js" as PathProbeConfigure
import "../../../modules/nodejs/nodejs.js" as NodeJs

NodeJsProbe {
    names: ["npm"]

    // Outputs
    property path npmBin
    property path npmRoot
    property path npmPrefix

    configure: {
        var result = PathProbeConfigure.configure(names, nameSuffixes, nameFilter, pathPrefixes,
                                                  pathSuffixes, platformPaths, environmentPaths,
                                                  platformEnvironmentPaths, qbs.pathListSeparator);
        result.npmBin = result.found ? NodeJs.findLocation(result.filePath, "bin") : undefined;
        result.npmRoot = result.found ? NodeJs.findLocation(result.filePath, "root") : undefined;
        result.npmPrefix = result.found ? NodeJs.findLocation(result.filePath, "prefix") : undefined;

        found = result.found;
        path = result.path;
        filePath = result.filePath;
        fileName = result.fileName;
        npmBin = result.npmBin;
        npmRoot = result.npmRoot;
        npmPrefix = result.npmPrefix;
    }
}
