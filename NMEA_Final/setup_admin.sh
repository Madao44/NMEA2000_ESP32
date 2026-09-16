#!/bin/bash
# Script tout-en-un a executer UNE FOIS en tant que root (sudo bash setup_admin.sh)
# sur le serveur nmea2k.obsea.es.
#
# Fait 2 choses :
#   1. Installe Docker et ajoute l'utilisateur nmea2k au groupe docker
#   2. Trouve automatiquement le vhost Apache HTTPS de nmea2k.obsea.es et y
#      ajoute un reverse proxy vers le futur conteneur FROST-Server (port 8080)
#
# Ne necessite AUCUNE recherche ni edition manuelle de fichier.

set -e

DOMAIN="nmea2k.obsea.es"
LINUX_USER="nmea2k"

echo "============================================"
echo " 1/2 - Installation de Docker"
echo "============================================"
if command -v docker >/dev/null 2>&1; then
    echo "Docker deja installe, on passe."
else
    curl -fsSL https://get.docker.com -o /tmp/get-docker.sh
    sh /tmp/get-docker.sh
fi
usermod -aG docker "$LINUX_USER"
echo "OK - Docker installe, utilisateur $LINUX_USER ajoute au groupe docker."
echo "($LINUX_USER devra fermer/rouvrir sa session SSH pour que ca prenne effet)"
echo ""

echo "============================================"
echo " 2/2 - Configuration du reverse proxy Apache"
echo "============================================"

PROXY_MARKER="FROST-Server reverse proxy"
PROXY_BLOCK="
    # --- ${PROXY_MARKER} (ajoute automatiquement le $(date +%Y-%m-%d)) ---
    ProxyPreserveHost On
    ProxyTimeout 15
    ProxyPass        /FROST-Server/ http://127.0.0.1:8080/FROST-Server/
    ProxyPassReverse /FROST-Server/ http://127.0.0.1:8080/FROST-Server/
"

echo "Recherche du fichier vhost HTTPS pour $DOMAIN ..."
FILE=$(grep -rlE "ServerName[[:space:]]+$DOMAIN|ServerAlias[[:space:]]+$DOMAIN" /etc/apache2/sites-enabled/ 2>/dev/null \
       | xargs grep -l "VirtualHost \*:443" 2>/dev/null | head -n1 || true)

if [ -z "$FILE" ]; then
    echo "ERREUR : impossible de trouver automatiquement le vhost HTTPS de $DOMAIN."
    echo "Lance 'apache2ctl -S' pour le localiser a la main."
    echo "(Docker a quand meme ete installe avec succes, cette partie a echoue seulement.)"
    exit 1
fi
echo "Fichier trouve : $FILE"

if grep -q "$PROXY_MARKER" "$FILE"; then
    echo "Le bloc proxy est deja present dans ce fichier, rien a faire."
else
    BACKUP="$FILE.bak.$(date +%Y%m%d%H%M%S)"
    cp "$FILE" "$BACKUP"
    echo "Sauvegarde creee : $BACKUP"

    awk -v block="$PROXY_BLOCK" '
        { lines[NR] = $0 }
        /<\/VirtualHost>/ { last = NR }
        END {
            for (i = 1; i <= NR; i++) {
                if (i == last) print block
                print lines[i]
            }
        }
    ' "$FILE" > "$FILE.tmp" && mv "$FILE.tmp" "$FILE"

    echo "Bloc proxy insere dans $FILE (juste avant </VirtualHost>)."
fi

echo "Activation des modules Apache proxy + proxy_http ..."
a2enmod proxy proxy_http >/dev/null

echo "Verification de la configuration Apache ..."
if apache2ctl configtest; then
    echo "Configuration OK, rechargement d'Apache ..."
    systemctl reload apache2
    echo ""
    echo "============================================"
    echo " TERMINE AVEC SUCCES"
    echo "============================================"
    echo "Docker est installe et $DOMAIN est pret a recevoir le proxy FROST-Server."
    echo "$LINUX_USER peut maintenant continuer seul (docker compose up -d, etc.)"
else
    echo ""
    echo "ERREUR : configuration Apache invalide apres modification."
    if [ -n "${BACKUP:-}" ]; then
        cp "$BACKUP" "$FILE"
        echo "Fichier original restaure depuis $BACKUP."
    fi
    echo "Apache n'a PAS ete rechargee. Le site continue de fonctionner normalement."
    exit 1
fi
