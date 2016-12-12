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

import qbs 1.0
import qbs.Process
import qbs.FileInfo

Probe {
    // Inputs
    property string executable: 'pkg-config'
    property string name
    property string minVersion
    property string exactVersion
    property string maxVersion
    property stringList libDirs // Full, non-sysrooted paths, mirroring the environment variable

    // Output
    property stringList cflags
    property stringList libs
    property string modversion

    configure: {
        if (!name)
            throw '"name" must be specified';
        var p = new Process();
        try {
            var args = [ name ];
            if (minVersion !== undefined)
                args.push(name + ' >= ' + minVersion);
            if (exactVersion !== undefined)
                args.push(name + ' = ' + exactVersion);
            if (maxVersion !== undefined)
                args.push(name + ' <= ' + maxVersion);
            var libDirsToSet = libDirs;
            if (qbs.sysroot) {
                p.setEnv("PKG_CONFIG_SYSROOT_DIR", qbs.sysroot);
                if (!libDirsToSet) {
                    libDirsToSet = [
                        qbs.sysroot + "/usr/lib/pkgconfig",
                        qbs.sysroot + "/usr/share/pkgconfig"
                    ];
                }
            }
            if (libDirsToSet)
                p.setEnv("PKG_CONFIG_LIBDIR", libDirsToSet.join(qbs.pathListSeparator));
            if (p.exec(executable, args.concat([ '--cflags' ])) === 0) {
                cflags = p.readStdOut().trim();
                cflags = cflags ? cflags.split(/\s/) : [];
                if (p.exec(executable, args.concat([ '--libs' ])) === 0) {
                    libs = p.readStdOut().trim();
                    libs = libs ? libs.split(/\s/) : [];
                    if (p.exec(executable, args.concat([ '--modversion' ])) === 0) {
                        modversion = p.readStdOut().trim();
                        found = true;
                        console.info("PkgConfigProbe: found library " + name);
                        return;
                    }
                }
            }
            found = false;
            cflags = undefined;
            libs = undefined;
        } finally {
            p.close();
        }
    }
}
