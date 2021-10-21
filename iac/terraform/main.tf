resource "kubernetes_pod" "localization-netapp" {
  metadata {
    name = "localization-netapp"
    namespace = "evolved5g"
    labels = {
      app = "LocalizationNetApp"
    }
  }

  spec {
    container {
      image = "dockerhub.hi.inet/evolved-5g/localization-netapp:latest"
      name  = "LocalizationNetApp"
    }
  }
}

resource "kubernetes_service" "localization-netapp_service" {
  metadata {
    name = "localization-netapp-service"
    namespace = "evolved5g"
  }
  spec {
    selector = {
      app = kubernetes_pod.example.metadata.0.labels.app
    }
    port {
      port = 1191
      target_port = 1191
    }
  }
}
