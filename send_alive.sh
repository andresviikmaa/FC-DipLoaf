#/bin/bash
(hostname && date && iwconfig && ifconfig) > /tmp/status.txt
curl -X PUT --data-binary @/tmp/status.txt http://robotiina.zed.ee/ip.php 