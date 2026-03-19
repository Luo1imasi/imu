#!/bin/bash
# Build roboto-imu Debian package
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PACKAGE="roboto-imu"
VERSION="1.0.0"
ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/roboparty"
DEB_DIR="${PACKAGE}_${VERSION}_${ARCH}"

# Source ROS 2 if available (ament_cmake is optional)
for distro in jazzy iron humble rolling; do
    if [ -f "/opt/ros/${distro}/setup.bash" ]; then
        echo ">>> Sourcing ROS 2 ${distro}"
        source "/opt/ros/${distro}/setup.bash"
        break
    fi
done

echo ">>> Starting compilation..."
rm -rf build && mkdir -p build
pushd build > /dev/null
cmake .. \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DCMAKE_PREFIX_PATH="${PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release
make -j"$(nproc)"
DESTDIR="${SCRIPT_DIR}/build/destdir" cmake --install .
popd > /dev/null

echo ">>> Preparing Debian package structure..."
rm -rf "${DEB_DIR}" "${DEB_DIR}.deb"
mkdir -p "${DEB_DIR}/DEBIAN"

# Copy cmake installed files
if [ -d "build/destdir${PREFIX}" ]; then
    mkdir -p "${DEB_DIR}${PREFIX}"
    cp -a "build/destdir${PREFIX}/." "${DEB_DIR}${PREFIX}/"
fi

# Install init_imu.sh to /opt/roboparty/bin
mkdir -p "${DEB_DIR}${PREFIX}/bin"
cp init_imu.sh "${DEB_DIR}${PREFIX}/bin/"
chmod 755 "${DEB_DIR}${PREFIX}/bin/init_imu.sh"

# udev rules
if [ -d etc/udev/rules.d ]; then
    mkdir -p "${DEB_DIR}/etc/udev/rules.d"
    cp etc/udev/rules.d/*.rules "${DEB_DIR}/etc/udev/rules.d/"
fi

# Copy DEBIAN maintainer scripts
cp debian/postinst "${DEB_DIR}/DEBIAN/"
cp debian/postrm   "${DEB_DIR}/DEBIAN/"
[ -f debian/conffiles ] && cp debian/conffiles "${DEB_DIR}/DEBIAN/"
chmod 755 "${DEB_DIR}/DEBIAN/postinst" "${DEB_DIR}/DEBIAN/postrm"

# Generate Control file (Replace placeholders)
sed -e "s/ARCH_PLACEHOLDER/${ARCH}/g" \
    -e "s/VERSION_PLACEHOLDER/${VERSION}/g" \
    debian/control > "${DEB_DIR}/DEBIAN/control"

echo ">>> Executing dpkg-deb build..."
dpkg-deb --root-owner-group --build "${DEB_DIR}"

echo ">>> Success! Generated ${DEB_DIR}.deb"
