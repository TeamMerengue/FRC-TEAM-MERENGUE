// Desplazamiento suave al hacer clic en los enlaces
document.querySelectorAll('a[href^="#"]').forEach(enlace => {
    enlace.addEventListener('click', function(e) {
        e.preventDefault();
        const destino = document.querySelector(this.getAttribute('href'));
        destino.scrollIntoView({
            behavior: 'smooth'
        });
    });
});

// Validación del formulario de contacto
const formulario = document.querySelector('#contacto-form');

formulario.addEventListener('submit', function(e) {
    e.preventDefault();
    const nombre = document.querySelector('#nombre').value.trim();
    const email = document.querySelector('#email').value.trim();
    const mensaje = document.querySelector('#mensaje').value.trim();

    if (nombre === '' || email === '' || mensaje === '') {
        alert('Por favor, completa todos los campos.');
        return;
    }

    alert('¡Gracias por contactarnos, ${nombre}! Nos pondremos en contacto pronto.');
    formulario.reset();
});

// Efecto de aparición al hacer scroll (Scroll Reveal)
const elementosOcultos = document.querySelectorAll('.reveal');

const mostrarAlDesplazar = () => {
    const alturaPantalla = window.innerHeight;

    elementosOcultos.forEach(elemento => {
        const distanciaDesdeArriba = elemento.getBoundingClientRect().top;

        if (distanciaDesdeArriba < alturaPantalla - 100) {
            elemento.classList.add('visible');
        }
    });
};

window.addEventListener('scroll', mostrarAlDesplazar);
mostrarAlDesplazar();

// Función para desplazarse hacia la sección "contacto"
function irAbajo() {
    const contacto = document.getElementById("contacto");
    contacto.scrollIntoView({ behavior: "smooth" });
}




/*Carrusel*/
const btnLeft = document.querySelector(".btn-left"),
      btnRight = document.querySelector(".btn-right"),
      slider = document.querySelector("#slider"),
      sliderSection = document.querySelectorAll(".slider-section");


btnLeft.addEventListener("click", e => moveToLeft())
btnRight.addEventListener("click", e => moveToRight())

setInterval(() => {
    moveToRight()
}, 3000);


let operacion = 0,
    counter = 0,
    widthImg = 100 / sliderSection.length;

function moveToRight() {
    console.log('eror')
    if (counter >= sliderSection.length-1) {
        counter = 0;
        operacion = 0;
        slider.style.transform = `translate(-${operacion}%)`;
        slider.style.transition = "none";
        return;
    } 
    counter++;
    operacion = operacion + widthImg;
    slider.style.transform = `translate(-${operacion}%)`;
    slider.style.transition = "all ease .6s"
    
}  

function moveToLeft() {
    counter--;
    if (counter < 0 ) {
        counter = sliderSection.length-1;
        operacion = widthImg * (sliderSection.length-1)
        slider.style.transform = `translate(-${operacion}%)`;
        slider.style.transition = "none";
        return;
    } 
    operacion = operacion - widthImg;
    slider.style.transform = `translate(-${operacion}%)`;
    slider.style.transition = "all ease .6s"
    
    
}   


// Numero autónomo

   // Función que se ejecuta cuando se selecciona un país
   function actualizarPrefijo() {
    const select = document.getElementById("pais");
    const telefono = document.getElementById("telefono");

    // Obtener el prefijo del país seleccionado
    const prefijo = select.value;

    // Establecer el valor inicial del campo de teléfono con el prefijo
    telefono.value = prefijo + " ";  // Añadir el signo "+" y el prefijo

    // Llamar a la función para formatear el teléfono después de actualizar el prefijo
    formatoTelefono(telefono);
}

// Función para formatear el teléfono: agregar paréntesis, guiones, y límites de dígitos
function formatoTelefono(input) {
    let valor = input.value.replace(/\D/g, '');  // Eliminar todo lo que no sean números

    // Verifica si tiene al menos el prefijo y formatea el número
    if (valor.length > 1) {
        // Formato: prefijo (xxx)-xxx-xxxx
        valor = valor.replace(/^(\d{1,2})(\d{0,3})(\d{0,3})(\d{0,4})/, '$1($2)-$3-$4');
    }

    // Limitar a 3 dígitos dentro de los paréntesis, 3 después del primer guion y 4 después del segundo guion
    valor = valor.replace(/^(\d{1,2})(\d{3})(\d{0,3})(\d{0,4})$/, '$1($2)-$3-$4');
    
    // Establecer el valor del campo de teléfono con el formato adecuado
    input.value = valor;
}