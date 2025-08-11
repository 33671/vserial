# Maintainer: zhang <zhangzizhuo2@gmail.com>

pkgname=vserial
pkgver=1.0.0
pkgrel=1
pkgdesc="Virtual serial port kernel module with systemd service"
arch=('x86_64' 'aarch64')
url="https://github.com/33671/vserial"
license=('GPL-2.0-or-later')
depends=('dkms' 'systemd')
makedepends=('linux-headers' 'gcc')
source=("vserial::git+https://github.com/33671/vserial"
        "vserial.service"
        "dkms.conf"
        "vserialctl.c")
sha256sums=('SKIP' 'SKIP' 'SKIP' 'SKIP')

prepare() {
    cd "$srcdir/vserial"
}

package() {
    dkmsdir="$pkgdir/usr/src/vserial-${pkgver}"
    install -dm755 "$dkmsdir"
    cd "$srcdir/vserial"
    cp -a --parents *.c Makefile Kbuild 99-vserial.rules "$dkmsdir"
    cd "$srcdir"
    install -Dm644 dkms.conf "$dkmsdir/dkms.conf"
    install -Dm644 vserial.service "$pkgdir/usr/lib/systemd/system/vserial.service"
    install -Dm644 "$srcdir/vserial/99-vserial.rules" \
                   "$pkgdir/usr/lib/udev/rules.d/99-vserial.rules"
    install -Dm755 "$srcdir/vserialctl" "$pkgdir/usr/bin/vserialctl"
}
build() {
    cd "$srcdir/vserial"
    make -C /lib/modules/$(uname -r)/build M="$PWD" modules
    cd "$srcdir"
    gcc -o vserialctl "$srcdir/vserialctl.c"
}
post_install() {
    systemctl daemon-reload
    systemctl enable --now vserial.service
}

pre_remove() {
    systemctl disable --now vserial.service 2>/dev/null || true
    dkms remove "vserial/${pkgver}" --all 2>/dev/null || true
}
